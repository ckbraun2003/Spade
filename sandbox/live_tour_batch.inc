// THE BATCH CHAPTER (L64; the lead, 2026-11-11): chapter 4 of the live smoke, "new in this batch". NOT A HEADER and not
// a translation unit of its own: live_tour.cpp includes it, inside its anonymous namespace, after the editor chapter,
// because the tour's classes (Tour, the ledger) are that file's. Its steps are THIS batch's visible changes, named in
// the batch's READY by the merging realm and written here; the next batch replaces them (a step worth keeping is
// promoted into a standing chapter by a light edit). A batch with nothing visible says so on the card.
//
// The ledger is kSteps' "batch" entries; the poster is 04_batch.png.

[[nodiscard]] std::string batch_card(std::string_view label) {
    (void)label;
    return "4  New since batch 23";
}

void batch_tour(Tour& t, const fs::path& assets) {
    (void)t;
    (void)assets;
}
