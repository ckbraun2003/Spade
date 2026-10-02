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

remote_name="$1"

# stdin: <local ref> <local sha> <remote ref> <remote sha>
while read -r local_ref local_sha remote_ref remote_sha; do
    [ -z "$remote_ref" ] && continue

    branch="${remote_ref#refs/heads/}"

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
