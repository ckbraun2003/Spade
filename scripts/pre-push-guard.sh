#!/bin/sh
# scripts/pre-push-guard.sh -- refuse a push without the owner's word.
#
# WHY THIS REPOSITORY NEEDS ONE
# -----------------------------
# Spade was extracted from the Kat monorepo on 2026-09-28. Kat enforces "nothing
# leaves this box without the owner's direct word" MECHANICALLY, with a hook of
# this shape armed at .git/hooks/pre-push. On the day of the split this repository
# inherited the rule and not the enforcement -- so the same rule was a guard in
# one repo and a convention in the other, and this is the one that now holds the
# engine.
#
#   A RULE WITH A GUARD IN ONE REPOSITORY AND NO GUARD IN THE OTHER FAILS THE
#   FIRST TIME SOMEONE WORKS IN THE UNGUARDED ONE -- and it fails silently,
#   because a push that is not refused looks exactly like a push that was allowed.
#
# Kat's own version of this file records what it cost to learn: twenty-four
# commits reached origin/master unauthorised while the rule existed in prose and
# in roughly ten restated messages. The realm that pushed was following a stored
# step -- "push once green" -- that had been superseded and never updated. Reading
# a rule and holding a procedure are different faculties, and under load the
# procedure wins. That is why this is a hook and not a sentence.
#
# WHY A SEPARATE VARIABLE FROM KAT'S
# ----------------------------------
# Kat releases on KAT_PUSH_AUTHORIZED; this releases on SPADE_PUSH_AUTHORIZED.
# Deliberately NOT shared. One variable would mean that authorising a push in one
# repository silently authorises a push in the other from the same shell -- and
# these two repositories now sit side by side, worked on in the same session, with
# a remote each. The estate's rule is that an authorisation covers THE PUSH IT WAS
# ABOUT; a shared override would quietly widen every authorisation to two repos.
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
# The rule is about the BOX, not about master. Kat's first version protected only
# master and main, and a realm branch push exported twelve commits with no prompt,
# no list and no exit code. The asymmetry decides it: refusing an authorised push
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
    # has genuinely never left this box. Kat's version learned this the hard way:
    # using the bare local_sha for a new branch degenerated to FULL HISTORY and
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
    echo "  This repository holds the engine; Kat consumes it as a library, so a" >&2
    echo "  push here changes what every consumer resolves." >&2
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
    echo "  Do not export that variable, and do not reuse Kat's" >&2
    echo "  KAT_PUSH_AUTHORIZED -- the two repositories are authorised separately" >&2
    echo "  on purpose, so one word cannot release both." >&2
    echo "" >&2
    exit 1
done

exit 0
