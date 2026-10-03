<!-- Source copy. Cameron Braun approved this guide on 2026-10-03; the Project Oversight
     coordinator relayed it to the Spade lead, and the user confirmed adoption on 2026-10-02 (local).
     Keep this text as received. Spade's working rules live in STYLE.md and glossary.yml. -->

# Engineering Style Guide (Draft Spec)

Oct 3, 2026 · @Cameron Braun

## Purpose and scope

One style guide applies to all text in every project and repo. It applies to text that people write and to text that models write. The base is one shared rule set. Each area adds a short set of custom rules.

This guide applies to:

- Public docs, design specs and READMEs.
- Code comments and docstrings.
- Commit messages and PR text.
- Model output: agent chat replies, briefs, summaries and status lines.
- CLI messages, user interface text and log messages.

The guide is project-neutral. It contains no rules for one product or domain. Each project adds its own domain terms to its copy of the glossary. This spec is a draft overview. It shows the shape of the guide, not the full rule set.

## Base standards

Five sources make the base. Where two sources disagree, the higher row in this table wins.

| Priority | Source | Controls |
| --- | --- | --- |
| 1 | Project glossary | The one approved term for each concept |
| 2 | Simplified technical English (modeled on ASD-STE100) | Sentence length, voice, one meaning per word |
| 3 | RFC 2119 keywords | The meaning of MUST, SHOULD and MAY in specs |
| 4 | Google Developer Documentation Style Guide | Formatting, code samples, lists, headings, word choice |
| 5 | Diátaxis | Document type: tutorial, how-to, reference or explanation |

We do not license the full STE dictionary. The project glossary and a short list of approved verbs replace it.

## Core writing rules

These rules apply in every area unless the area table says otherwise.

1. Write procedural sentences of 20 words or fewer. Write descriptive sentences of 25 words or fewer.
2. Put one instruction in each sentence.
3. Use the active voice. Use the passive voice only when the actor is not known.
4. Use the imperative for instructions. Write "Set the timeout." Do not write "You should set the timeout."
5. Use one word for one meaning. Use the glossary term every time. Do not use synonyms for variety.
6. Use simple verb tenses: present, past and future.
7. Do not use phrasal verbs when a single verb exists. Write "start", not "fire up".
8. Put a maximum of three nouns in a noun cluster. Write "the cache expiry time", not "the user session cache expiry time setting".
9. Write paragraphs of six sentences or fewer. Put one topic in each paragraph.
10. Put warnings and cautions before the step they apply to.
11. Do not use idioms, slang, humor or emoji.
12. Give numbers with their units, for example "30 s" or "512 MB".

Example:

| Do not write | Write |
| --- | --- |
| Once the server has been spun up, you'll want to go ahead and connect the client. | Start the server. Then connect the client. |
| The retry count can be tweaked if things look off. | If requests fail, change the retry count. |

## Area rules

Each area keeps the core rules and adds the custom rules below. "Strict" means Vale fails the build. "Warn" means Vale reports the issue but does not fail the build.

| Area | Diátaxis type | Custom rules | Vale level |
| --- | --- | --- | --- |
| Public docs (API and library reference, user guides) | All four, one type per page | Full core rules. Start each page with what the reader can do after they read it. Every code sample must run. | Strict |
| Design specs and ADRs | Explanation, plus a requirements section | Use RFC 2119 keywords in capitals, only in the requirements section. Give each requirement an ID, for example `REQ-012`. One requirement per sentence. | Strict |
| Code comments and docstrings | Reference | Say why, not what. First line is one sentence of 15 words or fewer. Docstrings list parameters, return values, units and errors. Sentence fragments are allowed in inline comments. | Warn |
| Commit messages | None | Subject line: imperative, 50 characters or fewer, no period. Body: what changed and why, wrapped at 72 characters. | Warn |
| PR text | None | Use the repo PR template. Open with "Before" and "After". List how you tested the change. | Warn |
| Agent chat replies and briefs | None | Answer first. Then give the evidence: a file and line, a command output or a link. Put a maximum of five lines in a reply unless the user asks for more. State what you checked and what you only assume. | Not checked by Vale; enforced by agent instructions |
| CLI messages and logs | Reference | Errors say what failed, why, and what to do next. Do not blame the user. Start log lines with the component name. | Warn |
| User interface text (graphical apps, editors, web pages) | None | Buttons are a verb plus a noun, for example "Save file". Use sentence case. Labels have a maximum of four words. Tooltips are one sentence. | Strict |

## Glossary

Each project keeps its own copy of the glossary. The shared part holds general software terms. Each project adds its own domain terms. Each entry gives one approved term and the terms it replaces. Vale reads the "Do not use" column and flags those words. The entries below are examples only.

| Approved term | Meaning | Do not use |
| --- | --- | --- |
| sign in | Start an authenticated session | log in, logon, sign on |
| select | Choose an item in a user interface | click on, hit, pick |
| start / stop | Begin or end a process | spin up, kick off, fire up, kill |
| remove | Take an item out of a list; the item still exists | delete (for this meaning) |
| delete | Destroy an item so it cannot come back | remove, erase, nuke |
| configuration | The settings that control a program | config (in prose), setup, prefs |

Approved verbs follow the same rule: one verb for one action. For example, use "start", "stop", "set", "get", "add" and "remove".

## Enforcement

Two controls enforce the guide: Vale for files, and agent instructions for model output.

**Vale (files).** Vale runs in CI on Markdown, docstrings and UI string files. It uses three rule packages:

- `Google`: the published Google style rules.
- `STE`: custom rules for sentence length, passive voice, phrasal verbs and noun clusters.
- `Glossary`: a substitution rule built from the glossary "Do not use" column.

Each area sets its own level in `.vale.ini`, as the area table shows.

**Agent instructions (model output).** Vale does not see chat replies. So each repo's agent instruction file (for example `AGENTS.md` or `CLAUDE.md`) loads the style file at the start of every session. The style file gives the core rules, the glossary and the agent chat rules in a short form. A model checks its own reply against those rules before it sends it. The project oversight coordinator is exempt from these rules.

## Files and rollout

Each repo gets the same three files:

- `docs/style/STYLE.md`: the core rules and the area rules.
- `docs/style/glossary.yml`: the glossary, in a form that people and Vale can read.
- `.vale.ini` and `.vale/styles/`: the Vale configuration and rule packages.

Rollout steps:

1. The coordinator sends the guide to each project lead.
2. The docs owner in each repo writes the three files, adds the project's domain terms to the glossary, and links `STYLE.md` from the agent instruction file.
3. Vale runs in warn mode for all areas first. After the first round of fixes, strict areas go to strict.

**Decisions**

- Each project keeps its own copy of the glossary.
- The STE rules apply to all agents except the project oversight coordinator.
