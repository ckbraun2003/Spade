#!/bin/sh
# scripts/pre-push-guard.sh -- refuse a push without the owner's word.
#
# WHY A HOOK AND NOT A SENTENCE
# -----------------------------
# The rule is "nothing leaves this machine without the owner's direct word".
# Written down, it has failed before: a session following a stored step ("push
# once green") that had been superseded exported twenty-four commits while the
# rule sat in prose and in roughly ten restated messages. Reading a rule and
# holding a procedure are different faculties, and under load the procedure
# wins. A push that is not refused looks exactly like a push that was allowed,
# so the refusal has to be mechanical.
#
# WHY ITS OWN VARIABLE
# --------------------
# This releases on SPADE_PUSH_AUTHORIZED and nothing else. Other repositories
# worked from the same shell -- a consumer's, say -- use their own variables,
# deliberately not shared: an authorisation covers THE PUSH IT WAS ABOUT, and a
# shared override would quietly widen every authorisation to several repos.
#
# HOW TO PUSH WHEN THE OWNER HAS ACTUALLY SAID SO
# -----------------------------------------------
#   SPADE_PUSH_AUTHORIZED=1 git push origin master
#
# Set it for the single command, never export it. An exported variable is a stored
# procedure again, which is the defect this file exists to stop.
#
# EVERY BRANCH, NOT JUST master
# -----------------------------
# The rule is about the BOX, not about master. A guard that protected only master
# and main once let a realm branch push export twelve commits with no prompt, no
# list and no exit code. The asymmetry decides it: refusing an authorised push
# costs one variable on one command; allowing an unauthorised export is the thing
# the rule exists to prevent.
#
# Install: copy to .git/hooks/pre-push and chmod +x. The hook is per-checkout by
# nature -- whether a push is authorised is a property of this machine and this
# conversation, not of the repository -- so this tracked copy is the durable
# record and the installed copy is the enforcement. A tracked copy alone enforces
# NOTHING; verify with `ls -l .git/hooks/pre-push`.

set -e

# A BROKEN PIPE IS STILL A REFUSAL. `git push ... | head` closes the reader while
# this hook is still printing its list; the next write raises SIGPIPE, and Git for
# Windows lets a push through when its hook dies of a signal (2026-10-22: a push
# piped into head went through unrefused; it happened to be authorised). The trap
# turns that signal, and the other common ones, into exit 1. set -e above is
# load-bearing too: if git ever starts the hook with SIGPIPE already ignored, POSIX
# makes this trap a no-op, and it is the failing write under set -e that refuses.
trap 'exit 1' PIPE HUP INT TERM

remote_name="$1"

# A USER'S FOLDER STAYS OUT OF A PUBLIC REPOSITORY (Oversight, L-S34, 2026-11-23)
# -------------------------------------------------------------------------------
# A path into a user's profile folder names a real person's machine, and the count of them in the pushed tree must
# be 0. Two shapes, both drive-anchored, any case:
#   - the Windows form: a letter, a colon, either slash or a doubled backslash, the Users folder, either slash, a name;
#   - the MSYS and WSL form: /<letter>/Users/<name> and /mnt/<letter>/Users/<name> (the letter dir not glued to a name
#     before it, so `src/c/Users/x` is not one).
# This guard refuses otherwise. No real name is written here: the rule is the SHAPE of the path, and a name is let
# through only when it is on the allow-list below, so a new real name is refused without anyone having listed it.
# Two things, both run BEFORE the owner's word is looked at: the owner's word releases a push, it does not vouch
# for what is in it.
#   TREE    `git grep` at the pushed commit: no file of the tree may hold a path of that shape with a name off the
#           allow-list. REFUSED, the files named. This is what keeps the public count at 0.
#   RANGE   the commits this push would export (reachable from the pushed commit, from no remote-tracking ref)
#           whose diffs carry such a path, added or removed, by commit, file and direction. REPORTED, never
#           refused: a commit that adds one and a later commit that removes it leaves the tree clean and the history
#           not, and the push ask has to say so. (L-S34 made the range a report; a refusal would block the redactions.)
# THE ALLOW-LIST, and where each entry comes from (a name matches whole: `ada` does not let `adaline` through):
#   - the fakes the repository's own tests and docs use for a user that does not exist (found by grepping the tracked
#     tree on 2026-11-23): ada, ada-2, adam, someone, zed, me, t, you;
#   - the system's profile folders: public, default, all users;
#   - any name that does not START with a letter or a digit is a placeholder, not a person (a literal ellipsis, an
#     angle-bracket or percent-sign or dollar-sign placeholder, nothing at all).
# A bare `Users/<name>` fragment with no drive anchor is NOT matched, deliberately: too many false hits (docs and tests
# say "the Users folder" and show relative paths), so a caption of the shape is not a promise that no such text leaks.
# Nothing is excluded by file. The messages print file names and commits, never the path. A branch deletion exports
# nothing and is not looked at.
host_path_allow='ada,ada-2,adam,someone,zed,me,t,you,public,default,all users'
host_path_pattern='[A-Za-z]:[\\/]+Users[\\/]+[^\\/]*|(^|[^A-Za-z0-9_.~-])(/mnt)?/[A-Za-z]/Users/+[^\\/]*'
# awk: bad(line) is true when the line holds a path of that shape whose name is neither a placeholder nor on the list.
host_path_awk='
function allowed(name,   i, n, e, c, entries) {
    if (name == "" || name !~ /^[a-z0-9]/) return 1
    n = split(allow, entries, ",")
    for (i = 1; i <= n; i++) {
        e = entries[i]
        if (substr(name, 1, length(e)) != e) continue
        c = substr(name, length(e) + 1, 1)
        if (c == "" || c !~ /[a-z0-9_.~-]/) return 1
    }
    return 0
}
function scan(rest, re, strip,   m) {
    while (match(rest, re)) {
        m = tolower(substr(rest, RSTART, RLENGTH))
        rest = substr(rest, RSTART + RLENGTH)
        sub(strip, "", m)
        if (!allowed(m)) return 1
    }
    return 0
}
function bad(line) {
    # strings, not /regex/ literals: a regex literal passed to a function is evaluated against $0 first
    if (scan(line, "[A-Za-z]:[\\\\/]+[Uu][Ss][Ee][Rr][Ss][\\\\/]+[^\\\\/]*", "^[a-z]:[\\\\/]+users[\\\\/]+")) return 1
    return scan(line, "(^|[^A-Za-z0-9_.~-])(/[Mm][Nn][Tt])?/[A-Za-z]/[Uu][Ss][Ee][Rr][Ss]/+[^\\\\/]*", "^[^/]?(/mnt)?/[a-z]/users/+")
}'

host_path_check() {  # $1 = the pushed sha; returns 1 (and says why on stderr) when its tree holds such a path
    sha=$1
    # `|| true` on each: git grep exits 1 for "no match", which set -e would turn into a refusal for the wrong reason.
    range_hits=$(git log -p -U0 --no-color --format="%x01%h %s" "$sha" --not --remotes 2>/dev/null |
        awk -v allow="$host_path_allow" "$host_path_awk"'
             /^\001/ { c = substr($0, 2); next }
             /^(\+\+\+|---) / { if ($1 == "+++") f = substr($0, 7); next }
             /^[+-]/ && bad($0) { d = (substr($0, 1, 1) == "+") ? "adds" : "removes"; k = c SUBSEP f SUBSEP d; if (!seen[k]++) print "      " c "  (" f ", " d ")" }' || true)
    if [ -n "$range_hits" ]; then
        echo "" >&2
        echo "  NOTE (reported, not refused) -- commits this push would export whose diffs carry a user's profile path:" >&2
        printf '%s\n' "$range_hits" >&2
        echo "" >&2
    fi
    # git grep -o prints `<sha>:<file>:<the match>`; a file name holds no colon here.
    tree_hits=$(git grep -I -i -o -E "$host_path_pattern" "$sha" -- 2>/dev/null |
        awk -v allow="$host_path_allow" "$host_path_awk"'
             { i = index($0, ":"); rest = substr($0, i + 1); j = index(rest, ":"); f = substr(rest, 1, j - 1); m = substr(rest, j + 1)
               if (bad(m) && !seen[f]++) print f }' || true)
    [ -z "$tree_hits" ] && return 0
    echo "" >&2
    echo "  PUSH REFUSED -- a user's profile path is in the tree being pushed" >&2
    echo "" >&2
    echo "  This repository is public. A drive-letter or MSYS/WSL path into a user's folder, with a name that is not on the" >&2
    echo "  guard's allow-list, must appear nowhere in it (use %USERPROFILE% or a placeholder). The owner's word does" >&2
    echo "  not cover it. Files that hold one:" >&2
    printf '%s\n' "$tree_hits" | sed 's/^/      /' >&2
    echo "" >&2
    return 1
}

# stdin: <local ref> <local sha> <remote ref> <remote sha>
while read -r local_ref local_sha remote_ref remote_sha; do
    [ -z "$remote_ref" ] && continue

    branch="${remote_ref#refs/heads/}"

    if [ "$local_sha" != "0000000000000000000000000000000000000000" ]; then
        host_path_check "$local_sha" || exit 1
    fi

    [ -n "$SPADE_PUSH_AUTHORIZED" ] && continue

    # WHAT "WOULD EXPORT" MEANS. `local_sha --not --remotes` is commits reachable
    # from what is being pushed and reachable from NO remote-tracking ref -- what
    # has genuinely never left this box. An earlier version learned this the hard
    # way: using the bare local_sha for a new branch degenerated to FULL HISTORY and
    # printed 197 KB of stderr where the true answer was six commits. It never let
    # a push through; it failed toward ILLEGIBILITY, which defeats the reason this
    # prints a list rather than a count. Too much output and no output are both
    # illegible.
    if [ "$local_sha" = "0000000000000000000000000000000000000000" ]; then
        range=""
    else
        range="$local_sha --not --remotes"
    fi

    echo "" >&2
    echo "  PUSH REFUSED -- $remote_name/$branch" >&2
    echo "" >&2
    echo "  Nothing leaves this machine without the owner's direct word." >&2
    echo "  This repository holds the engine, and consumers build against it, so" >&2
    echo "  a push here changes what every consumer resolves." >&2
    echo "" >&2
    if [ -z "$range" ]; then
        echo "  This push DELETES $branch on $remote_name. It exports no commits," >&2
        echo "  and it destroys a ref someone may be working from." >&2
    else
        echo "  This push would export (commits on no remote-tracking ref):" >&2
        # Unquoted on purpose: $range carries `--not --remotes` as separate args.
        git log --oneline $range 2>/dev/null | sed 's/^/      /' >&2 || true
        echo "" >&2
        n=$(git rev-list --count $range 2>/dev/null || echo "?")
        echo "      ($n commit(s))" >&2
    fi
    echo "" >&2
    echo "  If the owner has said so for THIS push, run it once as:" >&2
    echo "      SPADE_PUSH_AUTHORIZED=1 git push $remote_name $branch" >&2
    echo "" >&2
    echo "  Do not export that variable, and do not reuse another repository's" >&2
    echo "  push variable -- each repository is authorised separately on purpose," >&2
    echo "  so one word cannot release two." >&2
    echo "" >&2
    exit 1
done

exit 0
