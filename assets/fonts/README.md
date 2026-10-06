# The UI font

The sandbox and the editor draw their UI in **Inter**: one regular and one semibold weight at one size. This follows the editor design's §13 (EDT-021) and the user's direction of 2026-10-06 for a sleek, Unity/Unreal-style UI.

| File | What |
|---|---|
| `Inter-Regular.ttf` | body text, labels and values |
| `Inter-SemiBold.ttf` | section headers |
| `OFL.txt` | the licence: SIL Open Font License 1.1, copyright 2016 The Inter Project Authors |

**Source:** Inter 4.1, <https://github.com/rsms/inter/releases/download/v4.1/Inter-4.1.zip>. The two files are copied unchanged from its `extras/ttf/`, and `OFL.txt` is its `LICENSE.txt`.

**SHA-256:**
- `Inter-4.1.zip`: `9883fdd4a49d4fb66bd8177ba6625ef9a64aa45899767dde3d36aa425756b11e`
- `Inter-Regular.ttf`: `40d692fce188e4471e2b3cba937be967878f631ad3ebbbdcd587687c7ebe0c82`
- `Inter-SemiBold.ttf`: `78a843fade9d4612a5567302fb595b56976eb5fcebf4fea5a5912d638bafcde3`

**How it is loaded:** the build copies this folder to `fonts/` beside `spade_sandbox`, and the window loads it from there at startup (`sandbox/ui_theme.hpp`). If a file is missing, the window says so on stderr and in its HUD, and the live smoke fails its `ui-font` check. It never falls back silently (L6).

The OFL allows bundling and redistribution with software. It requires the licence to travel with the font files, so `OFL.txt` stays beside them wherever they are copied.
