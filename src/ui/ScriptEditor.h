// ScriptEditor.h : the Scripts page's editor STATE (an App member — see the
// "page state stays an App member" rule in ui-file-layout.md). The behaviour
// lives in ui/pages/PageScripts.cpp; this header only names what it keeps
// between frames, so App.h can hold one `ScriptEditorState` instead of thirty
// loose fields.
//
// The editor is ImGui's InputTextMultiline with its text colour set to
// transparent, drawn OVER a syntax-coloured rendering of the same buffer that
// the page paints into the parent draw list first (a parent's draw list is
// rendered before its children, so the colour sits under the widget's own
// selection and cursor). The widget is sized to the WHOLE text and never
// scrolls itself; the child around it scrolls both, so gutter, colour and
// caret can only ever agree.
#pragma once

#include "imgui.h"

#include <string>
#include <vector>

namespace firedog {

// What a token looks like, decided once per text change by the Lua lexer.
enum class ScriptTokKind : unsigned char {
    Text,       // punctuation, brackets — the plain text colour
    Keyword,    // and break do else elseif end ... while
    Builtin,    // print, tonumber, string, math, table, pairs ...
    Fd,         // the member after "fd." — a command or a binding
    Number,
    String,     // short and long strings
    Comment,    // -- line and --[[ long ]] comments
    Operator,
    Ident,
};

struct ScriptTok {
    int           col  = 0;   // byte offset within its line
    int           len  = 0;   // bytes (a long token is split per line)
    ScriptTokKind kind = ScriptTokKind::Text;
};

// One lint or syntax finding, addressed by line so the right panel can jump.
struct ScriptIssue {
    int         line  = 0;    // 1-based
    int         col   = 0;    // byte offset within the line (0 when unknown)
    int         len   = 0;    // 0 = the whole line
    bool        error = false;   // false = warning
    std::string message;
};

// One completion candidate.
struct ScriptAcItem {
    std::string label;    // what the popup shows
    std::string insert;   // what accepting it puts in the buffer
    std::string help;     // one line, shown for the highlighted row
    int         back = 0; // caret steps back after insertion (1 puts it inside "{}")
};

struct ScriptEditorState {
    // ---- the buffer ----
    std::string path;     // file the buffer holds ("" = nothing loaded)
    std::string text;     // the InputText buffer (grown through the resize callback)
    // What the edit callback last saw. ImGui reverts the WHOLE edit session
    // when Escape is pressed inside an InputText (its "cancel"); an operator
    // reaching for Escape to close the completion popup must not lose ten
    // minutes of typing, so the page restores from here on that deactivation.
    std::string shadow;
    std::string saved;    // the file's contents at load / save: dirty = text != saved
    bool        crlf = false;   // the file used CRLF; Save writes it back that way
    unsigned    ver    = 1;     // bumped on every change to `text`
    unsigned    lexVer = 0;     // `ver` the token table was built from

    // ---- derived per text change (LexScript) ----
    float                               lexFontSize = 0.0f;   // widths depend on the font
    const void*                         lexFont     = nullptr;
    std::vector<int>                    lineStart;   // byte offset of each line (size = lines)
    std::vector<std::vector<ScriptTok>> toks;        // per line
    float                               maxLineW = 0.0f;   // widest line, pixels
    std::vector<std::string>            idents;      // sorted unique identifiers (completion)
    std::vector<ScriptIssue>            lint;        // fd.* names etc. (never the syntax error)

    // ---- syntax check (CheckLuaSyntax, debounced) ----
    std::string syntaxErr;          // "" = clean (once `checked`)
    int         syntaxLine = 0;
    bool        checked    = false;
    double      checkDue   = 0.0;   // ImGui time a check is due at; 0 = none pending

    // ---- caret, reported by the callback ----
    int  cursor     = 0;    // byte offset
    int  cursorLine = 0;    // 0-based
    int  cursorCol  = 0;    // byte offset within the line
    int  followLine = -1;   // >= 0: scroll the editor so this line is visible ...
    int  followCol  = 0;    // ... and this column (byte offset in the line)
    int  execShown  = 0;    // executing line the view last followed (edge detect)
    bool readOnly   = false;   // set per frame for the callback (orch_.Busy())

    // ---- completion popup ----
    bool acOpen  = false;
    bool acForce = false;   // Ctrl+Space: open even on an empty prefix
    bool acApply = false;   // a row was clicked: apply on the next callback
    int  acSel   = 0;
    int  acAnchor = 0;      // byte offset where the word being completed starts
    std::vector<ScriptAcItem> acItems;
    std::vector<ScriptAcItem> fdTable;   // every fd.* member: registry + LuaFdBuiltins
    ImVec2 acScreen = ImVec2(0, 0);      // where the popup goes (below the caret)
    ImVec2 acMin = ImVec2(0, 0), acMax = ImVec2(0, 0);   // the popup's rect last frame

    // ---- find ----
    char find[128] = "";
    int  selReq0 = -1, selReq1 = -1;   // selection to apply on the next callback
    bool focusReq = false;             // give the editor keyboard focus this frame
};

} // namespace firedog
