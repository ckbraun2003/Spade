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
    i=0
    while [ "$i" -lt 40 ]; do
        "$GIT" -C "$clone" commit -q --allow-empty -m "c$i" || exit 2
        i=$((i + 1))
    done
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

exit "$failed"
