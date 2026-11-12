"""The user guide's two build helpers (run by build.ps1, with the build venv's Python):

  stage  put the guide's pages, the shared release text and the screenshots in one folder for MkDocs
  print  put every page into one HTML file, in menu order, for Edge to print to a PDF

Nothing here fetches anything. The guide's pages are the only source: the install, requirements and release-notes
pages and the known-limits page come from packaging/common, so each text exists once.
"""
import argparse
import html
import os
import re
import shutil
import sys

import markdown
import yaml

SHARED = {  # packaging/common file -> the guide page it becomes
    "INSTALL.md": "install.md",
    "SYSTEM_REQUIREMENTS.md": "requirements.md",
    "RELEASE_NOTES.md": "release-notes.md",
}
LIMITS_FROM = "### What you should know before you start (known limits)"
LIMITS_TO = "### What is not in this release"
PLACEHOLDER = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540">'
    '<rect width="960" height="540" fill="#e8eaee" stroke="#9aa0aa" stroke-width="2" stroke-dasharray="10 8"/>'
    '<text x="480" y="250" font-family="Segoe UI, Arial, sans-serif" font-size="34" text-anchor="middle" fill="#3b4048">'
    "Picture to come</text>"
    '<text x="480" y="300" font-family="Consolas, monospace" font-size="24" text-anchor="middle" fill="#3b4048">{name}</text>'
    "</svg>\n"
)
IMAGE = re.compile(r"\(images/([A-Za-z0-9_.-]+)\)")


def read(path):
    with open(path, encoding="utf8", newline="") as f:
        return f.read().replace("\r\n", "\n")


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf8", newline="\n") as f:
        f.write(text)


def known_limits(release_notes):
    a = release_notes.index(LIMITS_FROM) + len(LIMITS_FROM)
    b = release_notes.index(LIMITS_TO)
    return (
        "# Known limits\n\n"
        "These are the things we already know about in this release. If one of them is worse than it is described here, "
        "that is worth noting.\n" + release_notes[a:b].rstrip() + "\n"
    )


def stage(args):
    guide = os.path.join(args.repo, "docs", "user-guide")
    common = os.path.join(args.repo, "packaging", "common")
    src = os.path.join(args.out, "src")
    if os.path.isdir(src):
        shutil.rmtree(src)
    os.makedirs(os.path.join(src, "images"))
    for name in sorted(os.listdir(guide)):
        if name.endswith(".md"):
            shutil.copyfile(os.path.join(guide, name), os.path.join(src, name))
    shutil.copyfile(os.path.join(guide, "mkdocs.yml"), os.path.join(args.out, "mkdocs.yml"))
    for packaged, page in SHARED.items():
        shutil.copyfile(os.path.join(common, packaged), os.path.join(src, page))
    write(os.path.join(src, "known-limits.md"), known_limits(read(os.path.join(common, "RELEASE_NOTES.md"))))

    taken = []
    if args.shots:
        for name in sorted(os.listdir(args.shots)):
            if name.lower().endswith(".png"):
                shutil.copyfile(os.path.join(args.shots, name), os.path.join(src, "images", name))
                taken.append(name)

    missing = []
    for name in sorted(os.listdir(src)):
        if not name.endswith(".md"):
            continue
        path = os.path.join(src, name)
        text = read(path)
        for image in sorted(set(IMAGE.findall(text))):
            if not os.path.exists(os.path.join(src, "images", image)):
                stem = os.path.splitext(image)[0]
                write(os.path.join(src, "images", stem + ".svg"), PLACEHOLDER.format(name=html.escape(image)))
                text = text.replace("(images/%s)" % image, "(images/%s.svg)" % stem)
                missing.append(image)
        write(path, text)

    confirm = []
    for name in sorted(os.listdir(src)):
        if name.endswith(".md"):
            for n, line in enumerate(read(os.path.join(src, name)).split("\n"), 1):
                if "[to confirm" in line or "[to be filled" in line:
                    confirm.append("%s:%d: %s" % (name, n, line.strip()))
    write(os.path.join(args.out, "figures-to-come.txt"), "".join(m + "\n" for m in sorted(set(missing))))
    write(os.path.join(args.out, "to-confirm.txt"), "".join(c + "\n" for c in confirm))
    print("staged: %d screenshots copied, %d pictures to come, %d lines to confirm" % (len(taken), len(set(missing)), len(confirm)))


def pages_in_order(mkdocs_yml):
    config = yaml.safe_load(read(mkdocs_yml))
    out = []

    def walk(items):
        for item in items:
            for title, value in item.items():
                if isinstance(value, list):
                    walk(value)
                else:
                    out.append((title, value))

    walk(config["nav"])
    return config, out


CSS = """
body { font-family: 'Segoe UI', Arial, sans-serif; font-size: 11pt; line-height: 1.45; color: #1d2026; margin: 0 auto; max-width: 17cm; }
h1 { font-size: 22pt; margin-top: 0; border-bottom: 2px solid #3b6fd4; padding-bottom: 4pt; }
h2 { font-size: 15pt; margin-top: 18pt; } h3 { font-size: 12pt; }
section { page-break-before: always; } section.first { page-break-before: avoid; }
img { max-width: 100%; border: 1px solid #c8ccd4; margin: 6pt 0; }
table { border-collapse: collapse; margin: 8pt 0; } th, td { border: 1px solid #c8ccd4; padding: 4pt 8pt; vertical-align: top; text-align: left; }
.admonition { border-left: 4px solid #3b6fd4; background: #eef3fc; padding: 4pt 12pt; margin: 10pt 0; }
.admonition-title { font-weight: bold; }
kbd, .keys kbd { font-family: Consolas, monospace; border: 1px solid #9aa0aa; border-bottom-width: 2px; border-radius: 3px; padding: 0 4pt; background: #f4f5f7; }
code { font-family: Consolas, monospace; background: #f1f2f4; padding: 0 2pt; }
.cover { text-align: center; margin-top: 6cm; } .cover h1 { border: none; font-size: 30pt; }
"""


def print_page(args):
    config, pages = pages_in_order(os.path.join(args.out, "mkdocs.yml"))
    src = os.path.join(args.out, "src")
    anchors = {path: "page-" + os.path.splitext(path)[0] for _, path in pages}
    extensions = ["admonition", "attr_list", "toc", "tables", "pymdownx.details", "pymdownx.keys"]
    parts = ['<section class="first cover"><h1>%s</h1><p>%s</p></section>' % (html.escape(config["site_name"]), html.escape(config.get("site_description", "")))]
    for title, path in pages:
        body = markdown.markdown(read(os.path.join(src, path)), extensions=extensions)
        body = re.sub(r'href="([a-z0-9-]+\.md)(#[^"]*)?"', lambda m: 'href="#%s"' % anchors.get(m.group(1), m.group(1)), body)
        parts.append('<section id="%s">%s</section>' % (anchors[path], body))
    document = (
        '<!doctype html><html lang="en"><head><meta charset="utf-8"><title>%s</title><style>%s</style></head><body>%s</body></html>\n'
        % (html.escape(config["site_name"]), CSS, "\n".join(parts))
    )
    write(os.path.join(args.out, "site", "print.html"), document)
    print("print page: %d pages" % len(pages))


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    s = sub.add_parser("stage")
    s.add_argument("--repo", required=True)
    s.add_argument("--out", required=True)
    s.add_argument("--shots")
    s.set_defaults(run=stage)
    p = sub.add_parser("print")
    p.add_argument("--out", required=True)
    p.set_defaults(run=print_page)
    args = parser.parse_args()
    args.run(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
