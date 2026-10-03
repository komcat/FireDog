// PageScripts.cpp : the Scripts page (View menu, full-page) — list, the Lua
// editor and the Lua debugger controls. Definitions of App members — see
// App.h; the editor's state is `ScriptEditorState` (ui/ScriptEditor.h).
//
// The editor (design-doc item C4) is ImGui's InputTextMultiline drawn with a
// TRANSPARENT text colour over a syntax-coloured rendering of the same buffer
// that this file paints into the parent draw list first. A parent's draw list
// is rendered before its child windows, so the colour sits under the widget's
// selection highlight and caret (ImGuiCol_InputTextCursor, not the text
// colour). The widget is sized to the WHOLE text and never scrolls itself;
// the child around it scrolls, so gutter, colour, find marks and caret share
// one scroll offset and cannot drift apart. Editing, selection, clipboard and
// undo come from ImGui; this file adds the lexer (colour + lint + completion
// candidates), the compile-only syntax check (LuaEngine::CheckLuaSyntax,
// debounced while typing), the completion popup (a tooltip-layer window so it
// draws above everything without taking keyboard focus), auto-indent, Tab
// indent, find, the breakpoint gutter and the executing-line highlight.
//
// The engine runs the FILE, never the buffer: Run is disabled while the buffer
// is dirty, so the line the debugger reports is always a line of the text on
// screen. Every write to the file goes through SaveScriptEditor.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "command/ExecutionContext.h"   // IsSystemScript / SystemScriptPath
#include "core/Log.h"
#include "core/DutSession.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <string_view>

namespace firedog {

namespace {

// ---- palette (token colours are this page's, not the theme's) ---------------
constexpr ImVec4 kColKeyword  = ImVec4(0.55f, 0.70f, 1.00f, 1.0f);
constexpr ImVec4 kColBuiltin  = ImVec4(0.86f, 0.86f, 0.67f, 1.0f);
constexpr ImVec4 kColFd       = ImVec4(0.40f, 0.85f, 0.90f, 1.0f);
constexpr ImVec4 kColNumber   = ImVec4(0.71f, 0.81f, 0.66f, 1.0f);
constexpr ImVec4 kColString   = ImVec4(0.85f, 0.60f, 0.47f, 1.0f);
constexpr ImVec4 kColComment  = ImVec4(0.45f, 0.62f, 0.38f, 1.0f);
constexpr ImVec4 kColOperator = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
constexpr ImVec4 kColIdent    = ImVec4(0.88f, 0.88f, 0.92f, 1.0f);
constexpr ImVec4 kColError    = ImVec4(1.00f, 0.40f, 0.30f, 1.0f);
constexpr ImVec4 kColWarn     = ImVec4(0.98f, 0.72f, 0.32f, 1.0f);
constexpr ImVec4 kColOk       = ImVec4(0.45f, 0.85f, 0.50f, 1.0f);

ImU32 TokColor(ScriptTokKind k)
{
    switch (k) {
    case ScriptTokKind::Keyword:  return ImGui::GetColorU32(kColKeyword);
    case ScriptTokKind::Builtin:  return ImGui::GetColorU32(kColBuiltin);
    case ScriptTokKind::Fd:       return ImGui::GetColorU32(kColFd);
    case ScriptTokKind::Number:   return ImGui::GetColorU32(kColNumber);
    case ScriptTokKind::String:   return ImGui::GetColorU32(kColString);
    case ScriptTokKind::Comment:  return ImGui::GetColorU32(kColComment);
    case ScriptTokKind::Operator: return ImGui::GetColorU32(kColOperator);
    case ScriptTokKind::Ident:    return ImGui::GetColorU32(kColIdent);
    case ScriptTokKind::Text:
    default:                      return ImGui::GetColorU32(ImGuiCol_Text);
    }
}

constexpr const char* kLuaKeywords[] = {
    "and", "break", "do", "else", "elseif", "end", "false", "for", "function", "goto",
    "if", "in", "local", "nil", "not", "or", "repeat", "return", "then", "true",
    "until", "while",
};

// The globals RunLuaFile opens (base, math, string, table) plus `fd`. Dotted
// entries are completion candidates for a library member; the bare names are
// what the lexer colours as Builtin. `os` and `io` are NOT opened — the lint
// says so when a script reaches for them.
constexpr const char* kLuaGlobals[] = {
    "fd", "print", "tostring", "tonumber", "type", "pairs", "ipairs", "select", "pcall",
    "xpcall", "error", "assert", "next", "rawget", "rawset", "rawequal", "setmetatable",
    "getmetatable", "unpack",
    "string", "string.format", "string.sub", "string.len", "string.rep", "string.find",
    "string.match", "string.gmatch", "string.gsub", "string.upper", "string.lower",
    "string.byte", "string.char",
    "math", "math.floor", "math.ceil", "math.abs", "math.max", "math.min", "math.sqrt",
    "math.huge", "math.pi", "math.fmod", "math.exp", "math.log", "math.random",
    "math.tointeger", "math.maxinteger",
    "table", "table.insert", "table.remove", "table.concat", "table.sort", "table.unpack",
};

bool IsKeyword(std::string_view w)
{
    for (const char* k : kLuaKeywords) if (w == k) return true;
    return false;
}

bool IsGlobalName(std::string_view w)
{
    for (const char* g : kLuaGlobals) if (w == g) return true;
    return false;
}

bool IsIdentStart(char c) { return std::isalpha(static_cast<unsigned char>(c)) || c == '_'; }
bool IsIdentChar(char c)  { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; }
bool IsDigit(char c)      { return std::isdigit(static_cast<unsigned char>(c)) != 0; }
bool IsXDigit(char c)     { return std::isxdigit(static_cast<unsigned char>(c)) != 0; }

bool StartsWith(std::string_view s, std::string_view p)
{
    return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
}

// "[" "="* "[" at i -> the level (count of '='), else -1.
int LongBracketOpen(const std::string& s, std::size_t i)
{
    if (i >= s.size() || s[i] != '[') return -1;
    std::size_t j   = i + 1;
    int         lvl = 0;
    while (j < s.size() && s[j] == '=') { ++lvl; ++j; }
    return (j < s.size() && s[j] == '[') ? lvl : -1;
}

// Just past the matching "]" "="*lvl "]" searched from `from`, or the end.
std::size_t LongBracketClose(const std::string& s, std::size_t from, int lvl)
{
    const std::string close = "]" + std::string(static_cast<std::size_t>(lvl), '=') + "]";
    const std::size_t p     = s.find(close, from);
    return p == std::string::npos ? s.size() : p + close.size();
}

// ---- the lexer: tokens per line + lint + identifiers ------------------------
// One pass over the whole buffer per text change (long strings and comments
// span lines, so a per-line lexer would colour them wrong). `fdNames` is what
// "fd." may be followed by: the registry's commands plus LuaFdBuiltins.
void LexLuaSource(const std::string& s, const std::set<std::string>& fdNames,
                  ScriptEditorState& ed)
{
    ed.lineStart.assign(1, 0);
    ed.toks.assign(1, std::vector<ScriptTok>());
    ed.lint.clear();
    std::set<std::string> idents;

    const std::size_t n = s.size();
    std::size_t       i = 0;
    std::string_view  tokA, tokB;   // the two previous tokens ("fd" "." <name>)

    // Emit [from, to) as `k`, split at newlines so every token stays inside
    // its line (the renderer works per line).
    auto emit = [&](std::size_t from, std::size_t to, ScriptTokKind k) {
        std::size_t a = from;
        while (a < to) {
            const std::size_t nl = s.find('\n', a);
            const std::size_t e  = (nl == std::string::npos || nl >= to) ? to : nl;
            if (e > a)
                ed.toks.back().push_back({static_cast<int>(a - ed.lineStart.back()),
                                          static_cast<int>(e - a), k});
            if (e < to) {   // a newline inside the token
                ed.lineStart.push_back(static_cast<int>(e + 1));
                ed.toks.emplace_back();
                a = e + 1;
            } else {
                a = e;
            }
        }
    };
    auto note = [&](std::size_t from, std::size_t to) {
        tokA = tokB;
        tokB = std::string_view(s.data() + from, to - from);
    };
    auto lintAt = [&](std::size_t from, std::size_t to, bool error, std::string msg) {
        ScriptIssue is;
        is.line    = static_cast<int>(ed.lineStart.size());
        is.col     = static_cast<int>(from - ed.lineStart.back());
        is.len     = static_cast<int>(to - from);
        is.error   = error;
        is.message = std::move(msg);
        ed.lint.push_back(std::move(is));
    };

    while (i < n) {
        const char c = s[i];
        if (c == '\n') {
            ed.lineStart.push_back(static_cast<int>(i + 1));
            ed.toks.emplace_back();
            ++i;
            continue;
        }
        if (c == ' ' || c == '\t' || c == '\r') { ++i; continue; }

        // -- comment, line or long
        if (c == '-' && i + 1 < n && s[i + 1] == '-') {
            std::size_t end;
            const int   lvl = LongBracketOpen(s, i + 2);
            if (lvl >= 0) {
                end = LongBracketClose(s, i + 2 + static_cast<std::size_t>(lvl) + 2, lvl);
            } else {
                const std::size_t nl = s.find('\n', i);
                end = nl == std::string::npos ? n : nl;
            }
            emit(i, end, ScriptTokKind::Comment);
            note(i, end);
            i = end;
            continue;
        }
        // [[ long string ]]
        if (c == '[') {
            const int lvl = LongBracketOpen(s, i);
            if (lvl >= 0) {
                const std::size_t end =
                    LongBracketClose(s, i + static_cast<std::size_t>(lvl) + 2, lvl);
                emit(i, end, ScriptTokKind::String);
                note(i, end);
                i = end;
                continue;
            }
        }
        // "short" / 'short' string — an unterminated one ends at the newline
        if (c == '"' || c == '\'') {
            std::size_t j = i + 1;
            while (j < n && s[j] != c && s[j] != '\n') {
                if (s[j] == '\\' && j + 1 < n && s[j + 1] != '\n') ++j;
                ++j;
            }
            if (j < n && s[j] == c) ++j;
            emit(i, j, ScriptTokKind::String);
            note(i, j);
            i = j;
            continue;
        }
        // number
        if (IsDigit(c) || (c == '.' && i + 1 < n && IsDigit(s[i + 1]))) {
            std::size_t j = i;
            if (c == '0' && i + 1 < n && (s[i + 1] == 'x' || s[i + 1] == 'X')) {
                j += 2;
                while (j < n && (IsXDigit(s[j]) || s[j] == '.' || s[j] == 'p' || s[j] == 'P' ||
                                 ((s[j] == '+' || s[j] == '-') &&
                                  (s[j - 1] == 'p' || s[j - 1] == 'P'))))
                    ++j;
            } else {
                while (j < n && (IsDigit(s[j]) || s[j] == '.' || s[j] == 'e' || s[j] == 'E' ||
                                 ((s[j] == '+' || s[j] == '-') &&
                                  (s[j - 1] == 'e' || s[j - 1] == 'E'))))
                    ++j;
            }
            emit(i, j, ScriptTokKind::Number);
            note(i, j);
            i = j;
            continue;
        }
        // identifier / keyword / fd member
        if (IsIdentStart(c)) {
            std::size_t j = i + 1;
            while (j < n && IsIdentChar(s[j])) ++j;
            const std::string_view w(s.data() + i, j - i);
            const bool afterDot = (tokB == ".");
            ScriptTokKind k = ScriptTokKind::Ident;
            if (afterDot && tokA == "fd") {
                k = ScriptTokKind::Fd;
                const std::string name(w);
                if (fdNames.count(name) == 0)
                    lintAt(i, j, true,
                           "fd." + name + " is not a command or a binding - the run "
                           "would fail on this line");
            } else if (!afterDot && IsKeyword(w)) {
                k = ScriptTokKind::Keyword;
            } else if (!afterDot && IsGlobalName(w)) {
                k = ScriptTokKind::Builtin;
            } else if (afterDot && (tokA == "string" || tokA == "math" || tokA == "table")) {
                k = ScriptTokKind::Builtin;
            } else if (!afterDot && (w == "os" || w == "io") && j < n && s[j] == '.') {
                lintAt(i, j, false,
                       "the " + std::string(w) + " library is not opened for scripts "
                       "(only base, math, string and table are)");
            }
            if (k == ScriptTokKind::Ident && w.size() >= 3) idents.insert(std::string(w));
            emit(i, j, k);
            note(i, j);
            i = j;
            continue;
        }
        // operator / punctuation (two- and three-char operators first)
        std::size_t j = i + 1;
        if ((c == '=' || c == '~' || c == '<' || c == '>') && j < n && s[j] == '=') ++j;
        else if (c == '.' && j < n && s[j] == '.') { ++j; if (j < n && s[j] == '.') ++j; }
        else if (c == ':' && j < n && s[j] == ':') ++j;
        else if (c == '/' && j < n && s[j] == '/') ++j;
        else if (c == '<' && j < n && s[j] == '<') ++j;
        else if (c == '>' && j < n && s[j] == '>') ++j;
        const bool punct = (c == '(' || c == ')' || c == '{' || c == '}' || c == '[' ||
                            c == ']' || c == ',' || c == ';');
        emit(i, j, punct ? ScriptTokKind::Text : ScriptTokKind::Operator);
        note(i, j);
        i = j;
    }

    ed.idents.assign(idents.begin(), idents.end());
}

// ---- InputText callback helpers ---------------------------------------------
// The callback sees ImGui's INTERNAL copy of the text (data->Buf); the user
// buffer (ed.text) is written back by ImGui when the widget returns. Anything
// the callback changes is mirrored into `shadow` here so the Escape guard and
// the lexer agree with what the operator sees.
void MarkEdited(ImGuiInputTextCallbackData* d, ScriptEditorState& ed)
{
    ed.shadow.assign(d->Buf, static_cast<std::size_t>(d->BufTextLen));
    ++ed.ver;
    ed.checkDue = ImGui::GetTime() + 0.6;
}

int LineStartOf(const char* buf, int pos)
{
    while (pos > 0 && buf[pos - 1] != '\n') --pos;
    return pos;
}

// Does this line open a block, so the next one indents one step further?
bool OpensBlock(std::string_view line)
{
    // Drop a trailing comment and whitespace.
    const std::size_t cmt = line.find("--");
    if (cmt != std::string_view::npos) line = line.substr(0, cmt);
    while (!line.empty() && (line.back() == ' ' || line.back() == '\t' || line.back() == '\r'))
        line.remove_suffix(1);
    if (line.empty()) return false;
    auto endsWithWord = [&](std::string_view w) {
        if (line.size() < w.size() || line.compare(line.size() - w.size(), w.size(), w) != 0)
            return false;
        return line.size() == w.size() || !IsIdentChar(line[line.size() - w.size() - 1]);
    };
    if (endsWithWord("then") || endsWithWord("do") || endsWithWord("else") ||
        endsWithWord("repeat") || line.back() == '{' || line.back() == '(')
        return true;
    // "function name(...)" / "local function f(...)" / "= function(...)"
    if (line.back() == ')' && line.find("function") != std::string_view::npos) return true;
    return false;
}

void AcClose(ScriptEditorState& ed)
{
    ed.acOpen = false;
    ed.acItems.clear();
    ed.acSel = 0;
}

// Accept the highlighted candidate: replace [anchor, caret) with its text.
void AcAccept(ImGuiInputTextCallbackData* d, ScriptEditorState& ed)
{
    if (ed.acItems.empty()) { AcClose(ed); return; }
    const int sel = std::clamp(ed.acSel, 0, static_cast<int>(ed.acItems.size()) - 1);
    const ScriptAcItem it = ed.acItems[static_cast<std::size_t>(sel)];   // copy: cleared below
    const int cur = d->CursorPos;
    int       a   = ed.acAnchor;
    if (a < 0 || a > cur) a = cur;
    d->DeleteChars(a, cur - a);
    d->InsertChars(a, it.insert.c_str());
    const int c = a + static_cast<int>(it.insert.size()) - it.back;
    d->CursorPos = d->SelectionStart = d->SelectionEnd = c;
    AcClose(ed);
    MarkEdited(d, ed);
}

// Rebuild the candidate list from the word before the caret. Runs on every
// edit (typing) and on Ctrl+Space (`acForce`, which also accepts an empty
// prefix, e.g. right after "fd.").
void AcRefresh(ImGuiInputTextCallbackData* d, ScriptEditorState& ed)
{
    const bool force = ed.acForce;
    ed.acForce = false;
    ed.acItems.clear();
    ed.acSel = 0;
    const char* buf = d->Buf;
    const int   cur = d->CursorPos;
    int         s   = cur;
    while (s > 0 && (IsIdentChar(buf[s - 1]) || buf[s - 1] == '.')) --s;
    const std::string_view word(buf + s, static_cast<std::size_t>(cur - s));

    if (StartsWith(word, "fd.")) {
        const std::string_view prefix = word.substr(3);
        ed.acAnchor = s + 3;
        for (const ScriptAcItem& it : ed.fdTable)
            if (StartsWith(it.label, prefix) && it.label != prefix) ed.acItems.push_back(it);
    } else if (!word.empty() && IsIdentStart(word[0]) && (word.size() >= 2 || force)) {
        ed.acAnchor = s;
        for (const char* k : kLuaKeywords)
            if (StartsWith(k, word) && word != k) ed.acItems.push_back({k, k, "keyword", 0});
        for (const char* g : kLuaGlobals)
            if (StartsWith(g, word) && word != g) ed.acItems.push_back({g, g, "Lua", 0});
        for (const std::string& id : ed.idents)
            if (StartsWith(id, word) && id != word)
                ed.acItems.push_back({id, id, "in this script", 0});
    } else {
        ed.acAnchor = s;
    }
    if (ed.acItems.size() > 60) ed.acItems.resize(60);
    ed.acOpen = !ed.acItems.empty();
}

// Keys the callback reacts to. `edited` = this is the CallbackEdit event (an
// Enter has already put its '\n' in the buffer; a typed char is already in).
// ImGui fires ONE callback per frame — Completion, else Edit, else Always —
// so both Edit and Always route through here.
void HandleKeys(ImGuiInputTextCallbackData* d, ScriptEditorState& ed, bool edited)
{
    // Pending selection (find / jump), pending click-accept.
    if (ed.selReq0 >= 0) {
        const int len = d->BufTextLen;
        const int a   = std::clamp(ed.selReq0, 0, len);
        const int b   = std::clamp(ed.selReq1, 0, len);
        d->SelectionStart = a;
        d->SelectionEnd = d->CursorPos = b;
        ed.selReq0 = ed.selReq1 = -1;
        AcClose(ed);
    }
    if (ed.acApply) {
        ed.acApply = false;
        if (!ed.readOnly) AcAccept(d, ed);
        else AcClose(ed);
    }

    const bool enter = ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
                       ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false);
    const bool newlineJustTyped =
        edited && enter && d->CursorPos > 0 && d->Buf[d->CursorPos - 1] == '\n';

    if (ed.acOpen && !ed.readOnly) {
        const int n = static_cast<int>(ed.acItems.size());
        // Up / Down walk the list. The widget has already moved the caret a
        // line by the time we run; put it back where it was.
        if (n > 0 && ImGui::IsKeyPressed(ImGuiKey_DownArrow) && !ImGui::GetIO().KeyCtrl) {
            ed.acSel = (ed.acSel + 1) % n;
            d->CursorPos = d->SelectionStart = d->SelectionEnd =
                std::clamp(ed.cursor, 0, d->BufTextLen);
            return;
        }
        if (n > 0 && ImGui::IsKeyPressed(ImGuiKey_UpArrow) && !ImGui::GetIO().KeyCtrl) {
            ed.acSel = (ed.acSel + n - 1) % n;
            d->CursorPos = d->SelectionStart = d->SelectionEnd =
                std::clamp(ed.cursor, 0, d->BufTextLen);
            return;
        }
        if (newlineJustTyped) {   // Enter accepts: take the '\n' back out first
            d->DeleteChars(d->CursorPos - 1, 1);
            d->CursorPos = d->SelectionStart = d->SelectionEnd = d->CursorPos - 1;
            AcAccept(d, ed);
            return;
        }
    } else if (newlineJustTyped && !ed.readOnly) {
        // Auto-indent: copy the previous line's leading whitespace, one step
        // more when that line opened a block.
        const int   cur = d->CursorPos;
        const int   pe  = cur - 1;                 // the '\n'
        const int   ps  = LineStartOf(d->Buf, pe);
        std::string indent;
        for (int p = ps; p < pe && (d->Buf[p] == ' ' || d->Buf[p] == '\t'); ++p)
            indent += d->Buf[p];
        if (OpensBlock(std::string_view(d->Buf + ps, static_cast<std::size_t>(pe - ps))))
            indent += "  ";
        if (!indent.empty()) {
            d->InsertChars(cur, indent.c_str());
            d->CursorPos = d->SelectionStart = d->SelectionEnd =
                cur + static_cast<int>(indent.size());
        }
    }

    if (!ed.readOnly && (edited || ed.acForce)) AcRefresh(d, ed);

    // The caret left the word being completed: close.
    if (ed.acOpen) {
        const int cur = d->CursorPos;
        bool inWord = cur >= ed.acAnchor;
        for (int p = ed.acAnchor; inWord && p < cur; ++p)
            if (!IsIdentChar(d->Buf[p]) && d->Buf[p] != '.') inWord = false;
        if (!inWord) AcClose(ed);
    }
}

// Tab: accept a suggestion, else indent (a multi-line selection indents every
// line it touches; otherwise spaces to the next two-column stop).
void HandleTab(ImGuiInputTextCallbackData* d, ScriptEditorState& ed)
{
    if (ed.readOnly) return;
    if (ed.acOpen && !ed.acItems.empty()) { AcAccept(d, ed); return; }
    if (d->HasSelection()) {
        const int s = std::min(d->SelectionStart, d->SelectionEnd);
        const int e = std::max(d->SelectionStart, d->SelectionEnd);
        if (std::memchr(d->Buf + s, '\n', static_cast<std::size_t>(e - s)) != nullptr) {
            const int ls  = LineStartOf(d->Buf, s);
            int       p   = ls;
            int       end = e;
            while (p < end) {
                d->InsertChars(p, "  ");
                end += 2;
                const char* nl = static_cast<const char*>(
                    std::memchr(d->Buf + p, '\n', static_cast<std::size_t>(end - p)));
                if (!nl) break;
                p = static_cast<int>(nl - d->Buf) + 1;
            }
            d->SelectionStart = ls;
            d->SelectionEnd = d->CursorPos = end;
            MarkEdited(d, ed);
            return;
        }
        d->DeleteChars(s, e - s);
        d->CursorPos = d->SelectionStart = d->SelectionEnd = s;
    }
    const int cur = d->CursorPos;
    const int col = cur - LineStartOf(d->Buf, cur);
    const int n   = 2 - (col % 2);
    d->InsertChars(cur, "  ", "  " + n);
    d->CursorPos = d->SelectionStart = d->SelectionEnd = cur + n;
    MarkEdited(d, ed);
}

int ScriptEditorCallback(ImGuiInputTextCallbackData* d)
{
    ScriptEditorState& ed = *static_cast<ScriptEditorState*>(d->UserData);
    switch (d->EventFlag) {
    case ImGuiInputTextFlags_CallbackResize:
        // The std::string IS the buffer (imgui_stdlib's contract).
        ed.text.resize(static_cast<std::size_t>(d->BufTextLen));
        d->Buf = ed.text.data();
        return 0;
    case ImGuiInputTextFlags_CallbackCompletion:
        HandleTab(d, ed);
        break;
    case ImGuiInputTextFlags_CallbackEdit:
        HandleKeys(d, ed, true);
        // Escape is ImGui's "cancel": the buffer we see now is the REVERTED
        // one. Leave the shadow at the last real edit so the page can restore.
        if (!ImGui::IsKeyPressed(ImGuiKey_Escape, false))
            ed.shadow.assign(d->Buf, static_cast<std::size_t>(d->BufTextLen));
        ++ed.ver;
        ed.checkDue = ImGui::GetTime() + 0.6;
        break;
    case ImGuiInputTextFlags_CallbackAlways:
        HandleKeys(d, ed, false);
        break;
    default:
        return 0;
    }
    // Caret bookkeeping (every event that is not the resize).
    const int cur = std::clamp(d->CursorPos, 0, d->BufTextLen);
    int line = 0, ls = 0;
    for (int i = 0; i < cur; ++i) {
        if (d->Buf[i] == '\n') { ++line; ls = i + 1; }
    }
    if (cur != ed.cursor) {
        ed.followLine = line;
        ed.followCol  = cur - ls;
    }
    ed.cursor     = cur;
    ed.cursorLine = line;
    ed.cursorCol  = cur - ls;
    return 0;
}

// Case-insensitive find of `needle` in `hay` from `from` (forward) — npos if none.
std::size_t FindCI(const std::string& hay, const std::string& needle, std::size_t from)
{
    if (needle.empty() || hay.size() < needle.size()) return std::string::npos;
    auto eq = [](char a, char b) {
        return std::tolower(static_cast<unsigned char>(a)) ==
               std::tolower(static_cast<unsigned char>(b));
    };
    for (std::size_t i = from; i + needle.size() <= hay.size(); ++i) {
        bool hit = true;
        for (std::size_t k = 0; k < needle.size() && hit; ++k) hit = eq(hay[i + k], needle[k]);
        if (hit) return i;
    }
    return std::string::npos;
}

std::string SanitizeScriptName(const char* raw)
{
    std::string out;
    for (const char* p = raw; *p; ++p) {
        const char c = *p;
        if (IsIdentChar(c) || c == '-') out += c;
        else if (c == ' ') out += '_';
    }
    if (out.size() > 4 && out.compare(out.size() - 4, 4, ".lua") == 0) out.resize(out.size() - 4);
    return out;
}

} // namespace

// ---- list + selection -------------------------------------------------------

void App::LoadScriptsList()
{
    session_.LoadScriptsList();
    SyncScriptSelection();
}

void App::SelectScript(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(session_.Scripts().size())) return;
    const std::string& path = session_.Scripts()[static_cast<std::size_t>(idx)].path;
    if (path == scriptEd_.path) { scriptSel_ = idx; return; }
    if (ScriptDirty()) {   // guard: the buffer would be dropped
        scConfirmAction_ = 1;
        scPendingSel_    = idx;
        return;
    }
    scriptSel_      = idx;
    scriptBpAnchor_ = -1;
    LoadScriptEditor(path);
}

// ---- buffer <-> file ----------------------------------------------------------

void App::LoadScriptEditor(const std::string& path)
{
    ScriptEditorState& ed = scriptEd_;
    ed.path = path;
    ed.text.clear();
    std::ifstream f(path, std::ios::binary);
    if (f) {
        std::ostringstream ss;
        ss << f.rdbuf();
        ed.text = ss.str();
    } else {
        FD_WARN("script", "cannot read %s", path.c_str());
    }
    ed.crlf = ed.text.find("\r\n") != std::string::npos;
    if (ed.crlf) {   // edit LF, write back whatever the file used
        std::string lf;
        lf.reserve(ed.text.size());
        for (const char c : ed.text) if (c != '\r') lf += c;
        ed.text.swap(lf);
    }
    ed.saved  = ed.text;
    ed.shadow = ed.text;
    ++ed.ver;
    ed.checked   = false;
    ed.checkDue  = ImGui::GetTime();   // verify on the next draw
    ed.cursor = ed.cursorLine = ed.cursorCol = 0;
    ed.followLine = 0;
    ed.followCol  = 0;
    ed.execShown  = 0;
    ed.selReq0 = ed.selReq1 = -1;
    AcClose(ed);
}

bool App::SaveScriptEditor()
{
    ScriptEditorState& ed = scriptEd_;
    if (ed.path.empty()) return false;
    // A shipped script is the build's copy: a rebuild refreshes it and a release
    // replaces it. Saving one writes the MACHINE's copy instead, which the list
    // and ResolveScript then prefer by name. A standard procedure is never forked
    // (IsSystemScript), so an edit to one stays in the shipped file, to be
    // mirrored into the repo.
    if (IsShippedScript(ed.path) && !IsSystemScript(ed.path)) {
        const std::string dir = MachineScriptsDir();
        std::error_code   ec;
        std::filesystem::create_directories(dir, ec);
        const std::string to = dir + "/" + std::filesystem::path(ed.path).filename().string();
        FD_INFO("script", "%s is the shipped copy - saving this machine's copy %s instead",
                ed.path.c_str(), to.c_str());
        ed.path = to;
    }
    std::string out;
    if (ed.crlf) {
        out.reserve(ed.text.size() + ed.text.size() / 20);
        for (const char c : ed.text) {
            if (c == '\n') out += '\r';
            out += c;
        }
    } else {
        out = ed.text;
    }
    {
        std::ofstream f(ed.path, std::ios::binary | std::ios::trunc);
        if (!f) {
            FD_WARN("script", "cannot write %s", ed.path.c_str());
            return false;
        }
        f.write(out.data(), static_cast<std::streamsize>(out.size()));
        if (!f) {
            FD_WARN("script", "write failed: %s", ed.path.c_str());
            return false;
        }
    }
    ed.saved = ed.text;
    FD_INFO("script", "saved %s (%zu bytes)", ed.path.c_str(), out.size());
    LoadScriptsList();   // the description is the first line; keeps the selection by path
    VerifyScript();
    return true;
}

void App::RevertScriptEditor()
{
    ScriptEditorState& ed = scriptEd_;
    if (ed.path.empty()) return;
    ed.text   = ed.saved;
    ed.shadow = ed.saved;
    ++ed.ver;
    ed.checkDue = ImGui::GetTime();
    ed.selReq0 = ed.selReq1 = -1;
    AcClose(ed);
}

void App::VerifyScript()
{
    ScriptEditorState& ed = scriptEd_;
    const std::string  chunk = std::filesystem::path(ed.path).filename().string();
    ed.syntaxErr  = CheckLuaSyntax(ed.text, chunk.empty() ? "script" : chunk, &ed.syntaxLine);
    ed.checked    = true;
    ed.checkDue   = 0.0;
}

// The completion database: every fd.* member. Commands come from the registry
// (with their parameter names as the help line), the rest from the engine's
// own list — nothing here is typed twice.
void App::BuildScriptFdTable()
{
    ScriptEditorState& ed = scriptEd_;
    ed.fdTable.clear();
    for (const CommandSpec* c : orch_.registry.List()) {
        ScriptAcItem it;
        it.label  = c->name;
        it.insert = c->name + "{}";
        it.back   = 1;
        std::string params;
        for (const ParamSpec& p : c->params) {
            if (!params.empty()) params += ", ";
            params += p.name;
            if (!p.required) params += "?";
        }
        it.help = "fd." + c->name + "{ " + params + " }  -  " + c->description;
        ed.fdTable.push_back(std::move(it));
    }
    for (const LuaFdBuiltin& b : LuaFdBuiltins())
        ed.fdTable.push_back({b.name, b.insert, b.help, 0});
    std::sort(ed.fdTable.begin(), ed.fdTable.end(),
              [](const ScriptAcItem& a, const ScriptAcItem& b) { return a.label < b.label; });
}

void App::LexScript()
{
    ScriptEditorState& ed = scriptEd_;
    if (ed.fdTable.empty()) BuildScriptFdTable();
    std::set<std::string> fdNames;
    for (const ScriptAcItem& it : ed.fdTable) fdNames.insert(it.label);
    LexLuaSource(ed.text, fdNames, ed);

    // Widest line, so the widget can be sized to the text and never scroll
    // on its own. Depends on the font, hence the font stamp.
    ed.maxLineW = 0.0f;
    const char* buf = ed.text.c_str();
    const int   nl  = static_cast<int>(ed.lineStart.size());
    for (int i = 0; i < nl; ++i) {
        const int ls = ed.lineStart[static_cast<std::size_t>(i)];
        const int le = (i + 1 < nl) ? ed.lineStart[static_cast<std::size_t>(i) + 1] - 1
                                    : static_cast<int>(ed.text.size());
        ed.maxLineW = std::max(ed.maxLineW, ImGui::CalcTextSize(buf + ls, buf + le).x);
    }
    ed.lexVer      = ed.ver;
    ed.lexFontSize = ImGui::GetFontSize();
    ed.lexFont     = ImGui::GetFont();
}

// Select [col, col+len) on `line` (1-based; len 0 = the whole line), scroll
// there and give the editor the keyboard. The selection is applied by the
// callback on the next frame, which is why focus is requested with it.
void App::JumpToScriptLine(int line, int col, int len)
{
    ScriptEditorState& ed = scriptEd_;
    if (ed.lineStart.empty()) return;
    const int nl = static_cast<int>(ed.lineStart.size());
    const int li = std::clamp(line - 1, 0, nl - 1);
    const int ls = ed.lineStart[static_cast<std::size_t>(li)];
    const int le = (li + 1 < nl) ? ed.lineStart[static_cast<std::size_t>(li) + 1] - 1
                                 : static_cast<int>(ed.text.size());
    const int a = std::clamp(ls + col, ls, le);
    const int b = (len > 0) ? std::clamp(a + len, a, le) : le;
    ed.selReq0    = a;
    ed.selReq1    = b;
    ed.followLine = li;
    ed.followCol  = a - ls;
    ed.focusReq   = true;
}

// ---- the page ---------------------------------------------------------------

void App::DrawScriptsPanel()
{
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();
    ScriptEditorState& ed = scriptEd_;

    // Follow the list: a clean buffer tracks the selected file, a dirty one
    // stays (SelectScript raised the guard before the selection could move).
    if (scriptSel_ >= 0 && scriptSel_ < static_cast<int>(session_.Scripts().size())) {
        const std::string& want = session_.Scripts()[static_cast<std::size_t>(scriptSel_)].path;
        if (ed.path != want && !ScriptDirty()) LoadScriptEditor(want);
    } else if (!ed.path.empty() && !ScriptDirty()) {
        ed.path.clear();   // nothing selected any more (list emptied)
        ed.text.clear();
        ed.saved.clear();
        ed.shadow.clear();
        ++ed.ver;
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_S) && ScriptDirty() && !orch_.Busy())
        SaveScriptEditor();

    const float  stripH = ImGui::GetFrameHeightWithSpacing() * 3.2f;
    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  paneH  = avail.y - stripH;
    const float  leftW  = 230.0f;
    const float  rightW = 300.0f;

    // ---- Left: script list ----
    ImGui::BeginChild("scList", ImVec2(leftW, paneH), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Scripts");
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::SmallButton("Reload")) {
        if (ScriptDirty()) scConfirmAction_ = 3;
        else               LoadScriptsList();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(orch_.Busy() || ScriptDirty());
    if (ImGui::SmallButton("New")) {
        scNewOpen_     = true;
        scNewName_[0]  = '\0';
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && ScriptDirty())
        ImGui::SetTooltip("Save or revert the open script first");
    ImGui::TextDisabled("%s", session_.ScriptsDirUsed().c_str());
    ImGui::Separator();
    if (session_.Scripts().empty())
        ImGui::TextWrapped("No .lua scripts found. Create one with New, or drop files "
                           "in the folder and hit Reload.");
    for (int i = 0; i < static_cast<int>(session_.Scripts().size()); ++i) {
        const ScriptInfo& si = session_.Scripts()[static_cast<std::size_t>(i)];
        ImGui::PushID(i);
        const bool  isOpen = si.path == ed.path;
        std::string label  = si.name;
        if (isOpen && ScriptDirty()) label += " *";
        if (ImGui::Selectable(label.c_str(), scriptSel_ == i)) SelectScript(i);
        if (ImGui::IsItemHovered() && !si.description.empty())
            ImGui::SetTooltip("%s", si.description.c_str());
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Middle: the editor ----
    const float midW = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;
    ImGui::BeginChild("scSrc", ImVec2(midW, paneH), ImGuiChildFlags_Borders);
    if (!ed.path.empty()) {
        const ScriptInfo* si = nullptr;
        for (const ScriptInfo& s : session_.Scripts())
            if (s.path == ed.path) { si = &s; break; }
        const std::string fileName = std::filesystem::path(ed.path).filename().string();
        const bool        ro       = orch_.Busy();
        const bool        dirty    = ScriptDirty();

        if (si && !si->description.empty()) ImGui::TextWrapped("%s", si->description.c_str());
        if (!si)
            ImGui::TextColored(kColWarn, "%s is no longer in the list (product changed) - "
                                         "Save still writes it, Revert drops it",
                               fileName.c_str());
        // A standard procedure is run from the shipped copy wherever it is
        // launched from. Say so on the file being edited, because a product
        // copy opened here looks exactly like the one that runs, and editing
        // it would change nothing at all.
        if (IsSystemScript(ed.path)) {
            const std::string shipped = SystemScriptPath(ed.path);
            std::error_code   ec;
            if (!std::filesystem::equivalent(ed.path, shipped, ec))
                ImGui::TextColored(kColWarn,
                                   "STANDARD procedure - align_run and the "
                                   "page buttons are served from %s, never this copy",
                                   shipped.c_str());
            else
                ImGui::TextDisabled("standard procedure - kept in step with the "
                                    "align_ commands; a product cannot fork it");
        }
        if (IsShippedScript(ed.path)) {
            if (IsSystemScript(ed.path))
                ImGui::TextDisabled("shipped copy: a rebuild recopies the repo's config/scripts "
                                    "over it - mirror a durable edit back into the repo");
            else
                ImGui::TextDisabled("shipped copy: Save writes this machine's copy to %s, "
                                    "which then runs instead", MachineScriptsDir().c_str());
        }

        // Toolbar: Save / Revert / Verify | state | Find
        ImGui::BeginDisabled(ro || !dirty);
        if (ImGui::SmallButton("Save")) SaveScriptEditor();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Ctrl+S");
        ImGui::SameLine();
        ImGui::BeginDisabled(ro || !dirty);
        if (ImGui::SmallButton("Revert")) RevertScriptEditor();
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::SmallButton("Verify")) VerifyScript();
        ImGui::SameLine();
        if (ro)         ImGui::TextDisabled("read-only while a run is active");
        else if (dirty) ImGui::TextColored(kColWarn, "modified");
        else            ImGui::TextDisabled("saved");
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        if (!ed.checked) {
            ImGui::TextDisabled("syntax: not checked");
        } else if (ed.syntaxErr.empty()) {
            ImGui::TextColored(kColOk, "syntax OK");
        } else {
            ImGui::TextColored(kColError, "line %d: %s", ed.syntaxLine, ed.syntaxErr.c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("click to jump");
            if (ImGui::IsItemClicked()) JumpToScriptLine(ed.syntaxLine, 0, 0);
        }
        if (!ed.lint.empty()) {
            ImGui::SameLine();
            int errs = 0;
            for (const ScriptIssue& is : ed.lint) errs += is.error ? 1 : 0;
            ImGui::TextColored(errs ? kColError : kColWarn, "%d lint issue%s",
                               static_cast<int>(ed.lint.size()), ed.lint.size() == 1 ? "" : "s");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("listed on the right - click one to jump");
        }
        // Find, right-aligned.
        {
            const float findW = 170.0f;
            const float btnW  = ImGui::CalcTextSize("Prev").x + ImGui::CalcTextSize("Next").x +
                                ImGui::GetStyle().FramePadding.x * 4.0f +
                                ImGui::GetStyle().ItemSpacing.x * 3.0f;
            // Right-align on the current line when it fits, else a new line.
            const float need = findW + btnW + ImGui::CalcTextSize("Find").x +
                               ImGui::GetStyle().ItemSpacing.x;
            ImGui::SameLine();
            const float here  = ImGui::GetCursorPosX();
            const float right = here + ImGui::GetContentRegionAvail().x;
            if (right - need > here + 40.0f) ImGui::SameLine(right - need);
            else                             ImGui::NewLine();
            ImGui::TextDisabled("Find");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(findW);
            auto findStep = [&](bool forward) {
                const std::string needle = ed.find;
                if (needle.empty()) return;
                std::size_t hit = std::string::npos;
                if (forward) {
                    const std::size_t from = static_cast<std::size_t>(std::max(0, ed.cursor));
                    hit = FindCI(ed.text, needle, from);
                    if (hit == std::string::npos) hit = FindCI(ed.text, needle, 0);
                } else {
                    // Scan forward collecting the last hit before the selection.
                    const std::size_t before =
                        static_cast<std::size_t>(std::max(0, ed.cursor - static_cast<int>(needle.size())));
                    std::size_t p = FindCI(ed.text, needle, 0);
                    std::size_t last = std::string::npos;
                    while (p != std::string::npos) {
                        if (p >= before) break;
                        last = p;
                        p = FindCI(ed.text, needle, p + 1);
                    }
                    if (last == std::string::npos) {   // wrap: the last one in the file
                        p = FindCI(ed.text, needle, 0);
                        while (p != std::string::npos) { last = p; p = FindCI(ed.text, needle, p + 1); }
                    }
                    hit = last;
                }
                if (hit == std::string::npos) return;
                // Byte offset -> line/col through the line table.
                const int a = static_cast<int>(hit);
                int li = 0;
                while (li + 1 < static_cast<int>(ed.lineStart.size()) &&
                       ed.lineStart[static_cast<std::size_t>(li) + 1] <= a)
                    ++li;
                JumpToScriptLine(li + 1, a - ed.lineStart[static_cast<std::size_t>(li)],
                                 static_cast<int>(needle.size()));
            };
            if (ImGui::InputText("##scfind", ed.find, sizeof(ed.find),
                                 ImGuiInputTextFlags_EnterReturnsTrue))
                findStep(!ImGui::GetIO().KeyShift);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Enter: next   Shift+Enter: previous   (case-insensitive)");
            ImGui::SameLine();
            ImGui::BeginDisabled(ed.find[0] == '\0');
            if (ImGui::SmallButton("Prev")) findStep(false);
            ImGui::SameLine();
            if (ImGui::SmallButton("Next")) findStep(true);
            ImGui::EndDisabled();
        }
        ImGui::TextDisabled("gutter click: breakpoint | shift+click: range | Tab / Enter: "
                            "accept suggestion | Ctrl+Space: suggest | Tab: indent | Ctrl+S: save");
        ImGui::Separator();

        const bool thisRunning = running && si && st.scriptName == si->name;
        DrawScriptEditor(thisRunning, st.paused, thisRunning ? st.currentLine : 0);
    } else {
        ImGui::TextDisabled("Select a program on the left, or create one with New.");
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: run info + issues ----
    ImGui::BeginChild("scInfo", ImVec2(rightW, paneH), ImGuiChildFlags_Borders);
    if (st.paused)
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "State: Paused @ line %d",
                           st.currentLine);
    else
        ImGui::Text("State: %s", LuaEngine::StateName(st.state));
    if (!st.scriptName.empty()) ImGui::Text("Script: %s", st.scriptName.c_str());
    if (running && st.currentLine > 0 && !st.paused)
        ImGui::Text("Line: %d", st.currentLine);
    if (!st.lastCommand.empty()) ImGui::Text("Last command: %s", st.lastCommand.c_str());
    if (st.paused && !st.locals.empty()) {
        ImGui::Separator();
        ImGui::TextUnformatted("Locals");
        if (ImGui::BeginTable("##sclocals", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            for (const auto& [name, val] : st.locals) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(name.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(val.c_str());
            }
            ImGui::EndTable();
        }
    }
    if (!st.error.empty() && st.state != LuaEngine::State::Running)
        ImGui::TextColored(kColError, "%s", st.error.c_str());
    if (!st.ctx.empty()) {
        ImGui::Separator();
        ImGui::TextUnformatted("Context");
        if (ImGui::BeginTable("##scctx", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            for (const auto& [key, val] : st.ctx) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(key.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(CtxValueToString(val).c_str());
            }
            ImGui::EndTable();
        }
    }
    // Issues: the syntax error (one, from the compiler) and the lint (fd.*
    // names the engine would not bind). Click a row to jump to it.
    if (!ed.path.empty() && (!ed.lint.empty() || (ed.checked && !ed.syntaxErr.empty()))) {
        ImGui::Separator();
        ImGui::TextUnformatted("Issues");
        int n = 0;
        if (ed.checked && !ed.syntaxErr.empty()) {
            ImGui::PushID(n++);
            ImGui::PushStyleColor(ImGuiCol_Text, kColError);
            const std::string row = "syntax  L" + std::to_string(ed.syntaxLine) + ": " + ed.syntaxErr;
            if (ImGui::Selectable(row.c_str())) JumpToScriptLine(ed.syntaxLine, 0, 0);
            ImGui::PopStyleColor();
            ImGui::PopID();
        }
        for (const ScriptIssue& is : ed.lint) {
            ImGui::PushID(n++);
            ImGui::PushStyleColor(ImGuiCol_Text, is.error ? kColError : kColWarn);
            const std::string row = std::string(is.error ? "error   L" : "warn    L") +
                                    std::to_string(is.line) + ": " + is.message;
            if (ImGui::Selectable(row.c_str())) JumpToScriptLine(is.line, is.col, is.len);
            ImGui::PopStyleColor();
            ImGui::PopID();
        }
    }
    ImGui::EndChild();

    // ---- Bottom strip: Run / Stop ----
    ImGui::BeginChild("scStrip", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    const bool selValid = scriptSel_ >= 0 && scriptSel_ < static_cast<int>(session_.Scripts().size());
    const bool canRun   = !orch_.Busy() && selValid && !ScriptDirty();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Run", ImVec2(90.0f, 0.0f))) {
        LuaEngine::Env env = session_.MakeLuaEnv();
        scriptStartErr_.clear();
        const std::string& path = session_.Scripts()[static_cast<std::size_t>(scriptSel_)].path;
        orch_.lua.SetBreakpoints(scriptBps_[path]);
        if (!orch_.lua.Start(path, std::move(env), &scriptStartErr_))
            FD_WARN("script", "start rejected: %s", scriptStartErr_.c_str());
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && selValid && ScriptDirty() &&
        !orch_.Busy())
        ImGui::SetTooltip("Save first - a run executes the file on disk, not the buffer");
    ImGui::SameLine();
    if (!st.paused) {
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Pause", ImVec2(90.0f, 0.0f))) orch_.lua.Pause();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Continue", ImVec2(90.0f, 0.0f))) orch_.lua.Resume();
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(90.0f, 0.0f))) orch_.lua.StepLine();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(90.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();
    {
        const DutSession::Info di = DutSession::Instance().GetInfo();
        ImGui::SameLine();
        ImGui::TextDisabled("DUT: %s", di.open ? di.serial.c_str() : "none");
    }
    if (!scriptStartErr_.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(kColError, "start rejected: %s", scriptStartErr_.c_str());
    }
    ImGui::EndChild();

    DrawScriptCompletionPopup();
    DrawScriptModals();
}

// ---- the editor child ---------------------------------------------------------

void App::DrawScriptEditor(bool thisRunning, bool paused, int execLine)
{
    ScriptEditorState& ed = scriptEd_;
    const ImGuiStyle&  sty = ImGui::GetStyle();
    const bool followExec = thisRunning && !paused;
    ed.readOnly = orch_.Busy();
    if (ed.readOnly) AcClose(ed);

    // Derived tables: once per text change, or when the font changed.
    if (ed.lexVer != ed.ver || ed.lexFontSize != ImGui::GetFontSize() ||
        ed.lexFont != static_cast<const void*>(ImGui::GetFont()))
        LexScript();
    if (ed.checkDue > 0.0 && ImGui::GetTime() >= ed.checkDue) VerifyScript();

    // Follow the executing line when it moves (never while paused: the
    // operator may be reading elsewhere).
    if (followExec && execLine > 0 && execLine != ed.execShown) {
        ed.execShown  = execLine;
        ed.followLine = execLine - 1;
        ed.followCol  = 0;
    }

    ImGui::BeginChild("##sced", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);
    const float  lineH  = ImGui::GetTextLineHeight();
    const ImVec2 pad    = sty.FramePadding;
    const ImVec2 origin = ImGui::GetCursorScreenPos();   // content origin (scrolled)
    const ImVec2 winPos = ImGui::GetWindowPos();
    const ImVec2 winSz  = ImGui::GetWindowSize();
    const float  markW  = lineH;                         // breakpoint / arrow column
    const float  numW   = ImGui::CalcTextSize("00000").x;
    const float  gutterW = markW + numW + pad.x;
    const int    nLines  = std::max(1, static_cast<int>(ed.lineStart.size()));
    const ImVec2 availC  = ImGui::GetContentRegionAvail();
    const float  contentW = std::max(availC.x - gutterW, ed.maxLineW + pad.x * 2.0f + lineH * 4.0f);
    const float  contentH = std::max(availC.y, static_cast<float>(nLines) * lineH + pad.y * 2.0f);
    const ImVec2 textOrigin(origin.x + gutterW + pad.x, origin.y + pad.y);
    const char*  buf = ed.text.c_str();

    // Scroll so the requested line / column is in view (before drawing, so
    // this frame already shows it).
    if (ed.followLine >= 0) {
        const int   li   = std::clamp(ed.followLine, 0, nLines - 1);
        const float yTop = pad.y + static_cast<float>(li) * lineH;
        const float sy   = ImGui::GetScrollY();
        const float vh   = winSz.y;
        if (yTop < sy + lineH)               ImGui::SetScrollY(std::max(0.0f, yTop - lineH));
        else if (yTop + lineH > sy + vh - lineH * 1.5f)
            ImGui::SetScrollY(yTop + lineH * 2.5f - vh);
        const int   ls = ed.lineStart[static_cast<std::size_t>(li)];
        const int   le = (li + 1 < nLines) ? ed.lineStart[static_cast<std::size_t>(li) + 1] - 1
                                           : static_cast<int>(ed.text.size());
        const int   cc = std::clamp(ed.followCol, 0, std::max(0, le - ls));
        const float xc = gutterW + pad.x + ImGui::CalcTextSize(buf + ls, buf + ls + cc).x;
        const float sx = ImGui::GetScrollX();
        const float vw = winSz.x;
        if (xc - gutterW < sx + 8.0f)     ImGui::SetScrollX(std::max(0.0f, xc - gutterW - 40.0f));
        else if (xc > sx + vw - 40.0f)    ImGui::SetScrollX(xc - vw + 80.0f);
        ed.followLine = -1;
    }

    // Visible line range.
    int v0 = static_cast<int>(std::floor((winPos.y - textOrigin.y) / lineH));
    int v1 = static_cast<int>(std::ceil((winPos.y + winSz.y - textOrigin.y) / lineH));
    v0 = std::clamp(v0, 0, nLines - 1);
    v1 = std::clamp(v1, 0, nLines - 1);

    ImDrawList*    dl  = ImGui::GetWindowDrawList();
    std::set<int>& bps = scriptBps_[ed.path];
    const std::size_t findLen = std::strlen(ed.find);
    const ImU32 colNum    = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    const ImU32 colNumCur = ImGui::GetColorU32(ImGuiCol_Text);
    const ImU32 colBp     = IM_COL32(230, 70, 60, 255);
    const ImU32 colExec   = IM_COL32(255, 217, 77, 255);
    const std::string needle(ed.find);

    for (int i = v0; i <= v1; ++i) {
        const float y      = textOrigin.y + static_cast<float>(i) * lineH;
        const int   lineNo = i + 1;
        const int   ls     = ed.lineStart[static_cast<std::size_t>(i)];
        const int   le     = (i + 1 < nLines) ? ed.lineStart[static_cast<std::size_t>(i) + 1] - 1
                                              : static_cast<int>(ed.text.size());
        // Row tint: executing > syntax error > breakpoint > caret line.
        ImU32 bg = 0;
        if (execLine == lineNo)                                   bg = IM_COL32(255, 217, 77, 46);
        else if (ed.checked && !ed.syntaxErr.empty() && lineNo == ed.syntaxLine)
                                                                  bg = IM_COL32(255, 64, 64, 44);
        else if (bps.count(lineNo) != 0)                          bg = IM_COL32(255, 80, 80, 26);
        else if (i == ed.cursorLine)                              bg = IM_COL32(255, 255, 255, 12);
        if (bg)
            dl->AddRectFilled(ImVec2(origin.x, y),
                              ImVec2(origin.x + gutterW + contentW, y + lineH), bg);
        // Find marks.
        if (findLen > 0) {
            std::size_t p = FindCI(ed.text, needle, static_cast<std::size_t>(ls));
            while (p != std::string::npos && p + findLen <= static_cast<std::size_t>(le)) {
                const float x0 = textOrigin.x + ImGui::CalcTextSize(buf + ls, buf + p).x;
                const float x1 = x0 + ImGui::CalcTextSize(buf + p, buf + p + findLen).x;
                dl->AddRectFilled(ImVec2(x0, y), ImVec2(x1, y + lineH), IM_COL32(255, 200, 60, 70));
                p = FindCI(ed.text, needle, p + 1);
            }
        }
        // Tokens: advance x through the gaps so every glyph lands where the
        // widget would have drawn it.
        float x    = textOrigin.x;
        int   prev = 0;
        for (const ScriptTok& t : ed.toks[static_cast<std::size_t>(i)]) {
            if (t.col > prev) x += ImGui::CalcTextSize(buf + ls + prev, buf + ls + t.col).x;
            const char* a = buf + ls + t.col;
            const char* b = a + t.len;
            dl->AddText(ImVec2(x, y), TokColor(t.kind), a, b);
            x += ImGui::CalcTextSize(a, b).x;
            prev = t.col + t.len;
        }
        // Lint underlines.
        for (const ScriptIssue& is : ed.lint) {
            if (is.line != lineNo) continue;
            const int   c0 = std::clamp(is.col, 0, std::max(0, le - ls));
            const int   c1 = is.len > 0 ? std::clamp(is.col + is.len, c0, std::max(0, le - ls))
                                        : std::max(0, le - ls);
            const float x0 = textOrigin.x + ImGui::CalcTextSize(buf + ls, buf + ls + c0).x;
            const float x1 = textOrigin.x + ImGui::CalcTextSize(buf + ls, buf + ls + c1).x;
            dl->AddLine(ImVec2(x0, y + lineH - 1.0f), ImVec2(std::max(x1, x0 + 6.0f), y + lineH - 1.0f),
                        ImGui::GetColorU32(is.error ? kColError : kColWarn), 1.5f);
        }
        // Gutter: number, breakpoint disc, executing arrow.
        char num[16];
        std::snprintf(num, sizeof(num), "%d", lineNo);
        const float nw = ImGui::CalcTextSize(num).x;
        dl->AddText(ImVec2(origin.x + markW + numW - nw, y),
                    i == ed.cursorLine ? colNumCur : colNum, num);
        if (bps.count(lineNo) != 0)
            dl->AddCircleFilled(ImVec2(origin.x + markW * 0.5f, y + lineH * 0.5f),
                                lineH * 0.28f, colBp);
        if (execLine == lineNo) {
            const float cx = origin.x + markW * 0.5f, cy = y + lineH * 0.5f, r = lineH * 0.32f;
            dl->AddTriangleFilled(ImVec2(cx - r * 0.7f, cy - r), ImVec2(cx + r, cy),
                                  ImVec2(cx - r * 0.7f, cy + r), colExec);
        }
        // Gutter click: toggle a breakpoint (shift: range from the anchor).
        ImGui::SetCursorScreenPos(ImVec2(origin.x, y));
        ImGui::PushID(i);
        ImGui::InvisibleButton("##g", ImVec2(gutterW, lineH));
        if (ImGui::IsItemClicked()) {
            const bool bp = bps.count(lineNo) != 0;
            if (ImGui::GetIO().KeyShift && scriptBpAnchor_ > 0) {
                const int lo = std::min(scriptBpAnchor_, lineNo);
                const int hi = std::max(scriptBpAnchor_, lineNo);
                bool allSet = true;
                for (int l = lo; l <= hi; ++l) if (bps.count(l) == 0) { allSet = false; break; }
                for (int l = lo; l <= hi; ++l) {
                    if (allSet) bps.erase(l);
                    else        bps.insert(l);
                }
            } else {
                if (bp) bps.erase(lineNo);
                else    bps.insert(lineNo);
                scriptBpAnchor_ = lineNo;
            }
            if (thisRunning) orch_.lua.SetBreakpoints(bps);   // hot-swap live
        }
        ImGui::PopID();
    }

    // The widget itself: transparent text over the colour above, sized to the
    // whole text so the child (not the widget) is what scrolls.
    ImGui::SetCursorScreenPos(ImVec2(origin.x + gutterW, origin.y));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
    if (ed.focusReq) {
        ImGui::SetKeyboardFocusHere();
        ed.focusReq = false;
    }
    ImGuiInputTextFlags flags = ImGuiInputTextFlags_CallbackResize |
                                ImGuiInputTextFlags_CallbackEdit |
                                ImGuiInputTextFlags_CallbackAlways |
                                ImGuiInputTextFlags_CallbackCompletion;
    if (ed.readOnly) flags |= ImGuiInputTextFlags_ReadOnly;
    ImGui::InputTextMultiline("##sctext", ed.text.data(), ed.text.capacity() + 1,
                              ImVec2(contentW, contentH), flags, &ScriptEditorCallback, &ed);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);
    const bool edActive = ImGui::IsItemActive();

    // Escape guard: ImGui reverted the edit session on this deactivation —
    // put the operator's text back (see ScriptEditorState::shadow).
    if (ImGui::IsItemDeactivated() && !ed.readOnly && ed.text != ed.shadow) {
        ed.text = ed.shadow;
        ++ed.ver;
        ed.checkDue = ImGui::GetTime() + 0.2;
    }

    // Completion popup lifecycle. It stays open across the deactivation a
    // click on one of its rows causes (the mouse is over it), so the row can
    // be hit at all; anything else that takes the keyboard closes it.
    const ImVec2 mp = ImGui::GetIO().MousePos;
    const bool overAc = ed.acOpen && mp.x >= ed.acMin.x && mp.x <= ed.acMax.x &&
                        mp.y >= ed.acMin.y && mp.y <= ed.acMax.y;
    if (!edActive && !overAc && !ed.acApply) AcClose(ed);
    if (edActive && !ed.readOnly && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Space))
        ed.acForce = true;

    // Where the popup goes: under the caret.
    {
        const int   li = std::clamp(ed.cursorLine, 0, nLines - 1);
        const int   ls = ed.lineStart[static_cast<std::size_t>(li)];
        const int   le = (li + 1 < nLines) ? ed.lineStart[static_cast<std::size_t>(li) + 1] - 1
                                           : static_cast<int>(ed.text.size());
        const int   cc = std::clamp(ed.cursorCol, 0, std::max(0, le - ls));
        ed.acScreen = ImVec2(textOrigin.x + ImGui::CalcTextSize(buf + ls, buf + ls + cc).x,
                             textOrigin.y + static_cast<float>(li + 1) * lineH);
    }
    ImGui::EndChild();
}

// ---- the completion popup -------------------------------------------------------
// A window on the TOOLTIP layer: drawn above everything (the editor child, the
// other panels, a floating window) without taking keyboard focus when it
// appears — the keys keep going to the editor, which is what makes Up / Down /
// Tab / Enter work. A click on a row does take focus; the editor then asks for
// it back (`focusReq`) and applies the row on the next callback (`acApply`).
void App::DrawScriptCompletionPopup()
{
    ScriptEditorState& ed = scriptEd_;
    if (!ed.acOpen || ed.acItems.empty()) {
        ed.acMin = ed.acMax = ImVec2(0.0f, 0.0f);
        return;
    }
    const ImGuiViewport* vp    = ImGui::GetMainViewport();
    const float          lineH = ImGui::GetTextLineHeightWithSpacing();
    const float          maxW  = 560.0f;
    const int            kRows = 10;
    const int            n     = static_cast<int>(ed.acItems.size());
    const float          estH  = static_cast<float>(std::min(n, kRows) + 3) * lineH + 24.0f;
    ImVec2 pos = ed.acScreen;
    if (pos.x + maxW > vp->Pos.x + vp->Size.x) pos.x = std::max(vp->Pos.x, vp->Pos.x + vp->Size.x - maxW);
    if (pos.y + estH > vp->Pos.y + vp->Size.y) pos.y = std::max(vp->Pos.y, ed.acScreen.y - ImGui::GetTextLineHeight() - estH);
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSizeConstraints(ImVec2(320.0f, 0.0f), ImVec2(maxW, FLT_MAX));
    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_Tooltip | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking;
    if (ImGui::Begin("##scAc", nullptr, flags)) {
        ed.acSel = std::clamp(ed.acSel, 0, n - 1);
        const int first = std::clamp(ed.acSel - kRows / 2, 0, std::max(0, n - kRows));
        const int last  = std::min(n, first + kRows);
        for (int i = first; i < last; ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(ed.acItems[static_cast<std::size_t>(i)].label.c_str(), i == ed.acSel)) {
                ed.acSel    = i;
                ed.acApply  = true;
                ed.focusReq = true;
            }
            ImGui::PopID();
        }
        if (n > kRows) ImGui::TextDisabled("%d-%d of %d", first + 1, last, n);
        ImGui::Separator();
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + maxW - 24.0f);
        ImGui::TextDisabled("%s", ed.acItems[static_cast<std::size_t>(ed.acSel)].help.c_str());
        ImGui::PopTextWrapPos();
        ed.acMin = ImGui::GetWindowPos();
        ed.acMax = ImVec2(ed.acMin.x + ImGui::GetWindowSize().x, ed.acMin.y + ImGui::GetWindowSize().y);
    }
    ImGui::End();
}

// ---- modals: unsaved-changes guard, New Script ----------------------------------

void App::DrawScriptModals()
{
    ScriptEditorState& ed = scriptEd_;

    // Unsaved changes (Recipe Editor's shape). Raised by SelectScript (1),
    // SwitchPage (2) and Reload (3); never while orch_.Busy() — the editor is
    // read-only then, so nothing can become dirty mid-run in the first place.
    if (scConfirmAction_ != 0 && !ImGui::IsPopupOpen("Unsaved Script"))
        ImGui::OpenPopup("Unsaved Script");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Unsaved Script", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("%s has unsaved changes.",
                    std::filesystem::path(ed.path).filename().string().c_str());
        ImGui::Separator();
        auto proceed = [&]() {
            const int action = scConfirmAction_;
            scConfirmAction_ = 0;
            if (action == 1)      SelectScript(scPendingSel_);   // clean now: loads
            else if (action == 2) SwitchPage(scLeaveTarget_);
            else if (action == 3) LoadScriptsList();
            ImGui::CloseCurrentPopup();
        };
        if (ImGui::Button("Save", ImVec2(90.0f, 0.0f))) {
            if (SaveScriptEditor()) proceed();
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard", ImVec2(90.0f, 0.0f))) {
            RevertScriptEditor();
            proceed();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(90.0f, 0.0f))) {
            scConfirmAction_ = 0;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // New Script: a name, a file in the scanned folder, then open it.
    if (scNewOpen_ && !ImGui::IsPopupOpen("New Script")) ImGui::OpenPopup("New Script");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("New Script", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const std::string dir = session_.ScriptsDirUsed();
        ImGui::Text("New .lua file in %s", dir.c_str());
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::SetNextItemWidth(320.0f);
        const bool enter = ImGui::InputText("##scnewname", scNewName_, sizeof(scNewName_),
                                            ImGuiInputTextFlags_EnterReturnsTrue);
        const std::string name = SanitizeScriptName(scNewName_);
        const std::string path = dir + "/" + name + ".lua";
        std::error_code   ec;
        const bool exists = !name.empty() && std::filesystem::exists(path, ec);
        if (name.empty())  ImGui::TextDisabled("letters, digits, _ and - only");
        else if (exists)   ImGui::TextColored(kColWarn, "%s.lua already exists", name.c_str());
        else               ImGui::TextDisabled("%s", path.c_str());
        ImGui::Separator();
        ImGui::BeginDisabled(name.empty() || exists);
        if (ImGui::Button("Create", ImVec2(90.0f, 0.0f)) || (enter && !name.empty() && !exists)) {
            std::filesystem::create_directories(dir, ec);
            std::ofstream f(path, std::ios::binary | std::ios::trunc);
            if (f) {
                const std::string body = "-- " + name + ": what this script does\n\n"
                                         "fd.log(\"" + name + " started\")\n";
                f.write(body.data(), static_cast<std::streamsize>(body.size()));
                f.close();
                LoadScriptEditor(path);
                LoadScriptsList();   // finds the buffer's path and selects it
                FD_INFO("script", "created %s", path.c_str());
            } else {
                FD_WARN("script", "cannot create %s", path.c_str());
            }
            scNewOpen_ = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(90.0f, 0.0f))) {
            scNewOpen_ = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

} // namespace firedog
