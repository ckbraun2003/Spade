#!/bin/sh
# The pre-push guard (scripts/pre-push-guard.sh), the tracked copy, against
# throwaway bare repositories: every push without the owner's word is refused,
# however its output is read, and the one with it goes through.
#
#   sh tests/push_guard_test.sh <guard script> [git]
#
# Exit 0 when every case holds; otherwise 1, naming each case that didn't.
#
# WHY THE PIPED CASES. On Git for Windows a hook killed by a signal let the push
# through: `git push ... 2>&1 | head` closed the reader mid-list, SIGPIPE killed
# the hook, and the push went out unrefused (2026-10-22; the guard traps it
# since 9f6c206). On Linux git refuses a hook killed by a signal anyway, so the
# piped cases pass there with or without the trap; on Windows they fail
# against the guard before 9f6c206 (3fd0016~1).
#
# The installed copy (.git/hooks/pre-push) is the lead's push step's to check
# against this one, not a test's: it is per checkout, not in the repository.

set -u
guard=${1:?usage: push_guard_test.sh <guard script> [git]}
GIT=${2:-git}
[ -f "$guard" ] || { echo "push-guard-test: no guard at $guard"; exit 2; }
work=$(mktemp -d) || exit 2
trap 'cd / && rm -rf "$work"' EXIT

# Isolated from the user's and the system's git configuration: a global
# core.hooksPath would bypass the guard and pass every case for nothing.
GIT_CONFIG_NOSYSTEM=1
GIT_CONFIG_GLOBAL=/dev/null
HOME=$work
GIT_AUTHOR_NAME=push-guard-test
GIT_AUTHOR_EMAIL=push-guard-test@example.invalid
GIT_COMMITTER_NAME=$GIT_AUTHOR_NAME
GIT_COMMITTER_EMAIL=$GIT_AUTHOR_EMAIL
export GIT_CONFIG_NOSYSTEM GIT_CONFIG_GLOBAL HOME GIT_AUTHOR_NAME GIT_AUTHOR_EMAIL GIT_COMMITTER_NAME GIT_COMMITTER_EMAIL
unset SPADE_PUSH_AUTHORIZED

# Relative paths from inside the temp folder: Git for Windows clones an
# empty bare repository named by an MSYS /tmp path as "failed to iterate over
# objects/", where a relative path works on every platform.
cd "$work" || exit 2
remote=remote.git
clone=clone
failed=0

# A fresh remote holding master and a branch `gone` at one commit, and a clone
# forty commits ahead of it with the guard as its only hook.
setup() {
    rm -rf "$remote" "$clone" hooks
    "$GIT" init -q --bare "$remote" || exit 2
    "$GIT" clone -q "$remote" "$clone" 2>/dev/null || exit 2
    "$GIT" -C "$clone" commit -q --allow-empty -m base || exit 2
    "$GIT" -C "$clone" push -q origin HEAD:refs/heads/master HEAD:refs/heads/gone 2>/dev/null || exit 2
    # Forty empty commits from ONE git process (fast-import), not forty `git commit` processes: each setup() ran 40 of them, seven
    # setups a run, and on Windows a process costs a quarter second or more, so the test took 38 s alone and timed out at 120 s
    # under a loaded gate (2026-11-11). The commits are the same (empty, one parent each); the guard sees the same push.
    branch=$("$GIT" -C "$clone" symbolic-ref --short HEAD) || exit 2
    {
        i=0
        while [ "$i" -lt 40 ]; do
            echo "commit refs/heads/$branch"
            echo "committer $GIT_COMMITTER_NAME <$GIT_COMMITTER_EMAIL> $((1700000000 + i)) +0000"
            echo "data ${#i}"
            echo "$i"
            [ "$i" -eq 0 ] && echo "from refs/heads/$branch^0"
            echo
            i=$((i + 1))
        done
    } | "$GIT" -C "$clone" fast-import --quiet || exit 2
    mkdir -p hooks
    cp "$guard" hooks/pre-push
    chmod +x hooks/pre-push
    # Relative to where hooks run, the clone's root: an MSYS absolute path
    # here is one native git can't resolve, and the guard would never run.
    "$GIT" -C "$clone" config core.hooksPath ../hooks
}

# What the remote holds now: master's commit count, and whether `gone` and
# `realm` exist.
state() {
    n=$("$GIT" -C "$remote" rev-list --count refs/heads/master 2>/dev/null || echo 0)
    gone=no
    "$GIT" -C "$remote" rev-parse -q --verify refs/heads/gone >/dev/null && gone=yes
    realm=no
    "$GIT" -C "$remote" rev-parse -q --verify refs/heads/realm >/dev/null && realm=yes
    echo "master=$n gone=$gone realm=$realm"
}

# check <case> <the state it must leave>
check() {
    got=$(state)
    if [ "$got" = "$2" ]; then
        echo "push-guard-test: ok    $1 ($got)"
    else
        echo "push-guard-test: FAIL  $1: the remote holds $got, not $2"
        failed=1
    fi
}

untouched="master=1 gone=yes realm=no"

setup
"$GIT" -C "$clone" push origin HEAD:refs/heads/master 2>&1 | head -n 1 >/dev/null
check "refused, piped into a reader that stops at its first line" "$untouched"

setup
"$GIT" -C "$clone" push origin HEAD:refs/heads/master 2>&1 | true
check "refused, piped into a reader that reads nothing" "$untouched"

setup
"$GIT" -C "$clone" push origin HEAD:refs/heads/master >/dev/null 2>&1
check "refused, read whole" "$untouched"

setup
"$GIT" -C "$clone" push origin HEAD:refs/heads/master >/dev/null 2>&-
check "refused, its stderr closed" "$untouched"

setup
"$GIT" -C "$clone" push origin HEAD:refs/heads/realm >/dev/null 2>&1
check "refused, a new branch" "$untouched"

setup
"$GIT" -C "$clone" push origin :refs/heads/gone >/dev/null 2>&1
check "refused, a branch deleted" "$untouched"

setup
SPADE_PUSH_AUTHORIZED=1 "$GIT" -C "$clone" push origin HEAD:refs/heads/master 2>&1 | head -n 1 >/dev/null
check "let through with the owner's word, piped" "master=41 gone=yes realm=no"

# A USER'S PROFILE FOLDER (L-S34, 2026-11-23): a public repository must not receive a drive-letter path into one, with or without the owner's word, unless the
# name is on the guard's allow-list (the repository's own fakes, the system's profile folders, a placeholder). The tree is REFUSED; the range is only REPORTED.
# No real name is written in this file: the names below are made up for a person who does not exist and are on no list. Each case is its own push; the
# leak commits are made on the clone and undone with a hard reset, so one setup serves several.
REAL="Marlowe"
U="Users"  # the folder name, kept out of the file text so this file holds no path of the shape the guard refuses
leak_commit() {  # <file> <the text of the line>
    printf '%s\n' "$2" >"$clone/$1"
    "$GIT" -C "$clone" add "$1"
    "$GIT" -C "$clone" commit -q -m "add $1" || exit 2
}
authorized_push() {  # leaves git's stderr in err.txt
    SPADE_PUSH_AUTHORIZED=1 "$GIT" -C "$clone" push origin HEAD:refs/heads/master >/dev/null 2>err.txt
}
no_name_printed() {  # the refusal and the report name files and commits, never the path or the name
    if grep -qiE "$U[\\\\/]+${REAL}|${REAL}" err.txt; then echo "push-guard-test: FAIL  the output prints the name"; failed=1; fi
}

# Refused with the owner's word, whatever the spelling: either slash, a doubled backslash, another drive, lower case, a longer name that merely STARTS like a
# fake (adaline, tom), a real-looking name beside a fake on one line. The first is a real push, the rest run the guard directly, as git does (the push line on its
# stdin, the clone as the working directory): a real push costs seconds, and the matrix is a dozen cases.
guard_says() {  # runs the guard for the clone's HEAD as an authorised push; sets guard_rc, leaves the guard's stderr in err.txt
    head_sha=$("$GIT" -C "$clone" rev-parse HEAD)
    (cd "$clone" && printf 'refs/heads/master %s refs/heads/master %s\n' "$head_sha" "0000000000000000000000000000000000000000" |
        SPADE_PUSH_AUTHORIZED=1 sh ../hooks/pre-push origin) >/dev/null 2>"$work/err.txt"
    guard_rc=$?
}
setup
leak_commit notes.txt "see C:/$U/$REAL/Desktop/y"
authorized_push
check "refused with the owner's word in a real push, the tree holds a path with a name on no list" "$untouched"
grep -q "notes.txt" err.txt && echo "push-guard-test: ok    ... and the refusal names the file" || { echo "push-guard-test: FAIL  the refusal does not name notes.txt"; failed=1; }
no_name_printed
"$GIT" -C "$clone" reset -q --hard HEAD~1
for line in "see C:/$U/$REAL/Desktop/y" "see C:\\$U\\$REAL\\Desktop" "{\"p\":\"D:\\\\$U\\\\$REAL\\\\x\"}" "see c:/$(printf '%s' "$U" | tr 'A-Z' 'a-z')/$(printf '%s' "$REAL" | tr 'A-Z' 'a-z')/x" \
            "see /c/$U/$REAL/x" "see /C/$U/$REAL/x" "see /mnt/c/$U/$REAL/x" "(/d/$U/$REAL)" "see /c/$U/Adaline/x" \
            "see C:/$U/Adaline/x" "see C:/$U/Tom/x" "see C:/$U/Ada/x and C:/$U/$REAL/y" "see C:/$U/Ada-3/x"; do
    leak_commit notes.txt "$line"
    guard_says
    if [ "$guard_rc" = 1 ] && grep -q "notes.txt" err.txt; then echo "push-guard-test: ok    refused, naming the file: $line"; else echo "push-guard-test: FAIL  not refused (exit $guard_rc) or the file not named: $line"; failed=1; fi
    no_name_printed
    "$GIT" -C "$clone" reset -q --hard HEAD~1
done

# Let through with the owner's word: the repository's own fakes (any case), the system's profile folders, placeholders, and what is not a drive-letter path.
# One push carries every line (a push per line made this test twice as slow); if it is refused, each line is tried alone to name the culprit.
allowed_lines="C:/$U/Ada/x
C:/$U/Ada-2/x
C:\\$U\\Adam\\x
C:/$U/ada/x
C:/$U/adam/x
C:/$U/Someone/x
C:/$U/someone/x
C:/$U/Zed/x
C:/$U/me/x
C:/$U/t/x
C:/$U/you/x
C:/$U/.../x
C:/$U/Public/x
C:/$U/Default/x
C:/$U/All $U/x
C:/$U/<name>/x
C:/$U/%USERNAME%/x
C:/$U/\$USER/x
C:/$U/
the $U/$REAL folder, no drive letter
/c/$U/Ada/x and /mnt/c/$U/Public/x and /c/$U/<name>/x are MSYS and WSL paths with a fake, a system folder and a placeholder"
setup
leak_commit notes.txt "$allowed_lines"
authorized_push
check "let through with the owner's word: every fake, system folder, placeholder and non-drive-letter line, in one file" "master=42 gone=yes realm=no"
if [ "$(state)" != "master=42 gone=yes realm=no" ]; then
    printf '%s\n' "$allowed_lines" | while IFS= read -r line; do
        setup
        leak_commit notes.txt "$line"
        guard_says
        [ "$guard_rc" = 0 ] || echo "push-guard-test: FAIL  refused, but it must be let through: $line"
    done
fi

# The range is REPORTED, not refused: a path added by one commit and removed by the next leaves the tree clean, so the push goes through, and the report
# names both commits by short hash, the file and the direction (never the path).
setup
leak_commit notes.txt "see C:/$U/$REAL/Desktop/y"
added=$("$GIT" -C "$clone" rev-parse --short HEAD)
leak_commit notes.txt "see the user's folder"
removed=$("$GIT" -C "$clone" rev-parse --short HEAD)
authorized_push
check "let through with the owner's word, a path added by one commit and removed by the next (the tree is clean)" "master=43 gone=yes realm=no"
grep -q "NOTE (reported, not refused)" err.txt && echo "push-guard-test: ok    ... and the range is reported" || { echo "push-guard-test: FAIL  no range report"; failed=1; }
grep -q "$added .*(notes.txt, adds)" err.txt && echo "push-guard-test: ok    ... naming the commit that added it ($added)" || { echo "push-guard-test: FAIL  the report does not name $added as adding"; failed=1; }
grep -q "$removed .*(notes.txt, removes)" err.txt && echo "push-guard-test: ok    ... and the commit that removed it ($removed)" || { echo "push-guard-test: FAIL  the report does not name $removed as removing"; failed=1; }
no_name_printed

# A fake in the range is not reported.
setup
leak_commit notes.txt "see C:/$U/Ada/Desktop/y"
authorized_push
check "let through, a fake in the range" "master=42 gone=yes realm=no"
if grep -q "NOTE (reported" err.txt; then echo "push-guard-test: FAIL  a fake was reported"; failed=1; fi

# A redaction is let through: the remote already holds the path (pushed past the guard, as the old history was), and the push only removes it.
setup
leak_commit notes.txt "see C:/$U/$REAL/Desktop/y"
"$GIT" -C "$clone" push -q --no-verify origin HEAD:refs/heads/master >/dev/null 2>&1
check "(setup) the path is already on the remote" "master=42 gone=yes realm=no"
leak_commit notes.txt "see C:/<the user folder>/Desktop/y"
authorized_push
check "a redaction is let through with the owner's word" "master=43 gone=yes realm=no"

# Without the owner's word the push is refused, as before, whatever the tree holds.
setup
leak_commit notes.txt "clean"
"$GIT" -C "$clone" push origin HEAD:refs/heads/master >/dev/null 2>&1
check "refused without the owner's word, a clean tree" "$untouched"

# A deletion exports no commits and is not looked at for the path.
setup
leak_commit notes.txt "see C:/$U/$REAL/Desktop/y"
SPADE_PUSH_AUTHORIZED=1 "$GIT" -C "$clone" push origin :refs/heads/gone >/dev/null 2>&1
check "a branch deleted with the owner's word, a path in the clone's tree" "master=1 gone=no realm=no"

exit "$failed"
