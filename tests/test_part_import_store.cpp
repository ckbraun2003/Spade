#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "../tools/part_import/store.hpp"
#include "part_import_fixtures.hpp"
#include "unique_temp.hpp"
#include "world/mesh_file.hpp"
#include "world/sha256.hpp"

// spade_part_import's store (L88b; docs/design/interface/plans/2026-11-17-part-import-design.md, section 3): file names, the entry written
// whole by one rename, and the entry found again only when it is complete and its sidecars are what it says.

namespace {

namespace fs = std::filesystem;
using namespace spade;
using namespace spade::partimport;
using namespace spade::partimporttest;

// ---- file names ----------------------------------------------------------------------------------------------------------------

TEST(PartImportNames, ACharacterAFileNameCannotHoldBecomesAnUnderscore) {
    EXPECT_EQ(safe_file_name("arm left"), "arm left");
    EXPECT_EQ(safe_file_name("a/b:c*d?e\"f<g>h|i\\j"), "a_b_c_d_e_f_g_h_i_j");
    EXPECT_EQ(safe_file_name(std::string("x\x01y", 3)), "x_y");
    EXPECT_EQ(safe_file_name("caf\xc3\xa9"), "caf\xc3\xa9");
    EXPECT_EQ(safe_file_name("x. ."), "x");
    EXPECT_EQ(safe_file_name(""), "object");
    EXPECT_EQ(safe_file_name("..."), "object");
}

TEST(PartImportNames, AWindowsDeviceNameGetsALeadingUnderscore) {
    EXPECT_EQ(safe_file_name("con"), "_con");
    EXPECT_EQ(safe_file_name("NUL.txt"), "_NUL.txt");
    EXPECT_EQ(safe_file_name("com1"), "_com1");
    EXPECT_EQ(safe_file_name("LPT9"), "_LPT9");
    EXPECT_EQ(safe_file_name("console"), "console");
    EXPECT_EQ(safe_file_name("com10"), "com10");
}

TEST(PartImportNames, NamesThatMatchIgnoringCaseGetNumberedInOrder) {
    EXPECT_EQ(unique_file_names({"A", "a", "b", "A"}), (std::vector<std::string>{"A", "a-2", "b", "A-3"}));
    EXPECT_EQ(unique_file_names({"x/y", "x_y"}), (std::vector<std::string>{"x_y", "x_y-2"}));
    EXPECT_EQ(unique_file_names({"hub", "hub-2", "hub"}), (std::vector<std::string>{"hub", "hub-2", "hub-3"}));
    EXPECT_TRUE(unique_file_names({}).empty());
}

// ---- the entry -----------------------------------------------------------------------------------------------------------------

struct StoreRig {
    test::TempDir tmp{"part_import_store"};
    fs::path store = tmp.path() / "store";
    std::string source = "solid placeholder source bytes";
    KeyInputs inputs;
    std::vector<StoredObject> objects;

    StoreRig() {
        inputs.converter = "mesh-1";
        inputs.unit = "mm";
        objects.push_back({"arm left", box_mesh(2, 3, 5), {"bracket.stl: arm left", MeshUnit::millimetre, 0.0}});
        objects.push_back({"hub", box_mesh(1, 1, 1), {"bracket.stl: hub", MeshUnit::millimetre, 0.0}});
    }
    [[nodiscard]] std::string sha() const { return sha256_hex(source); }
    [[nodiscard]] std::string key() const { return cache_key(inputs); }
    [[nodiscard]] Expected<EntryRecord> write(std::string_view token = "t1") {
        return write_entry(store, source, "bracket.stl", inputs, objects, token);
    }
};

TEST(PartImportStore, AnEntryHoldsTheSourceTheSidecarsAndImportJsonUnderTheShaAndTheKey) {
    StoreRig rig;
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    const fs::path sha_dir = rig.store / rig.sha();
    EXPECT_EQ(rec->dir, sha_dir / rig.key());
    EXPECT_EQ(read_file(sha_dir / "bracket.stl"), rig.source);  // the source, as received
    EXPECT_TRUE(fs::is_regular_file(rec->dir / "import.json"));
    EXPECT_EQ(rec->sha256, rig.sha());
    EXPECT_EQ(rec->key, rig.key());
    EXPECT_EQ(rec->key_text, cache_key_text(rig.inputs));
    EXPECT_EQ(rec->converter, "mesh-1");
    EXPECT_EQ(rec->source_name, "bracket.stl");
    ASSERT_EQ(rec->sidecars.size(), 2u);
    EXPECT_EQ(rec->sidecars[0].object, "arm left");
    EXPECT_EQ(rec->sidecars[0].file, "arm left.spmesh");
    EXPECT_EQ(rec->sidecars[1].file, "hub.spmesh");
    EXPECT_EQ(rec->sidecars[0].triangles, 12u);
    EXPECT_NEAR(rec->sidecars[0].volume_m3, 30.0, 1e-9);
    for (std::size_t i = 0; i < 2; ++i) {
        const auto loaded = load_mesh_file(rec->dir / rec->sidecars[i].file);
        ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
        EXPECT_EQ(loaded->mesh->content_hash(), rig.objects[i].mesh->content_hash());
        EXPECT_EQ(loaded->provenance, rig.objects[i].provenance);
        char hex[17];
        std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(loaded->mesh->content_hash()));
        EXPECT_EQ(rec->sidecars[i].content_hash, hex);
    }
}

TEST(PartImportStore, ImportJsonSaysWhatTheEntryIs) {
    StoreRig rig;
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    const auto j = nlohmann::json::parse(read_file(rec->dir / "import.json"));
    EXPECT_EQ(j.at("import_version"), 2) << "rev 2: the entry carries each object's colour";
    EXPECT_EQ(j.at("sha256"), rig.sha());
    EXPECT_EQ(j.at("key"), rig.key());
    EXPECT_EQ(j.at("key_text"), cache_key_text(rig.inputs));
    EXPECT_EQ(j.at("converter"), "mesh-1");
    EXPECT_EQ(j.at("source"), "bracket.stl");
    ASSERT_EQ(j.at("objects").size(), 2u);
    EXPECT_EQ(j.at("objects")[0].at("object"), "arm left");
    EXPECT_EQ(j.at("objects")[0].at("file"), "arm left.spmesh");
    EXPECT_EQ(j.at("objects")[0].at("triangles"), 12);
}

TEST(PartImportStore, AFoundEntryIsTheOneThatWasWrittenAndNoTemporaryRemains) {
    StoreRig rig;
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    const auto found = find_entry(rig.store, rig.sha(), rig.key());
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, *rec);
    for (const auto& e : fs::directory_iterator(rig.store / rig.sha())) {
        EXPECT_NE(e.path().filename().string().rfind(".tmp-", 0), 0u) << e.path().string();
    }
}

TEST(PartImportStore, ThereIsNoEntryForAnotherKeyAnotherSourceOrAnEmptyStore) {
    StoreRig rig;
    EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value());  // nothing written yet
    ASSERT_TRUE(rig.write().has_value());
    KeyInputs other = rig.inputs;
    other.scale = 2.0;
    EXPECT_FALSE(find_entry(rig.store, rig.sha(), cache_key(other)).has_value());
    EXPECT_FALSE(find_entry(rig.store, sha256_hex("another source"), rig.key()).has_value());
}

TEST(PartImportStore, AnEntryWithoutImportJsonOrWithAMissingOrChangedSidecarIsNotFound) {
    {
        StoreRig rig;
        const auto rec = rig.write();
        ASSERT_TRUE(rec.has_value());
        fs::remove(rec->dir / "import.json");  // written last: without it the entry is incomplete
        EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value());
    }
    {
        StoreRig rig;
        const auto rec = rig.write();
        ASSERT_TRUE(rec.has_value());
        fs::remove(rec->dir / "hub.spmesh");
        EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value());
    }
    {
        StoreRig rig;
        const auto rec = rig.write();
        ASSERT_TRUE(rec.has_value());
        // another mesh's bytes under the first sidecar's name: the content hash is not the recorded one
        ASSERT_TRUE(save_mesh_file(rec->dir / "arm left.spmesh", *box_mesh(9, 9, 9)).has_value());
        EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value());
    }
    {
        StoreRig rig;
        const auto rec = rig.write();
        ASSERT_TRUE(rec.has_value());
        write_file(rec->dir / "import.json", "not json");
        EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value());
    }
}

TEST(PartImportStore, AnIncompleteDirectoryAtTheKeyIsReplacedAndACompleteOneIsKept) {
    StoreRig rig;
    const fs::path dir = entry_dir(rig.store, rig.sha(), rig.key());
    write_file(dir / "leftover.spmesh", "half an entry");  // a directory at the key with no import.json
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    EXPECT_FALSE(fs::exists(dir / "leftover.spmesh"));
    EXPECT_TRUE(find_entry(rig.store, rig.sha(), rig.key()).has_value());
    // a second write of the same entry leaves the complete one as it is
    write_file(rec->dir / "marker.txt", "kept");
    const auto again = rig.write("t2");
    ASSERT_TRUE(again.has_value()) << again.error().message;
    EXPECT_TRUE(fs::exists(rec->dir / "marker.txt"));
    EXPECT_EQ(*again, *rec);
}

TEST(PartImportStore, TwoObjectsWithTheSameNameGetTwoSidecars) {
    StoreRig rig;
    rig.objects.push_back({"HUB", box_mesh(4, 4, 4), {"bracket.stl: HUB", MeshUnit::millimetre, 0.0}});
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    ASSERT_EQ(rec->sidecars.size(), 3u);
    EXPECT_EQ(rec->sidecars[1].file, "hub.spmesh");
    EXPECT_EQ(rec->sidecars[2].file, "HUB-2.spmesh");
}

// ---- a sidecar as a store file of its own (the library contract's "Store files": <store>/<sha256>/<file>, sha256 of that file's bytes) -----------

TEST(PartImportStore, EachSidecarIsAlsoAStoreFileOfItsOwnAddressedByItsOwnBytes) {
    StoreRig rig;
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    ASSERT_EQ(rec->sidecars.size(), 2u);
    const auto j = nlohmann::json::parse(read_file(rec->dir / "import.json"));
    for (std::size_t i = 0; i < 2; ++i) {
        const std::string bytes = read_file(rec->dir / rec->sidecars[i].file);
        const std::string sha = sha256_hex(bytes);
        EXPECT_EQ(rec->sidecars[i].store_sha256, sha);
        EXPECT_EQ(read_file(rig.store / sha / rec->sidecars[i].file), bytes) << "the content-addressed copy";
        EXPECT_NE(sha, rig.sha());
        EXPECT_EQ(j.at("objects")[i].at("store_ref").at("sha256"), sha);
        EXPECT_EQ(j.at("objects")[i].at("store_ref").at("file"), rec->sidecars[i].file);
    }
    const auto found = find_entry(rig.store, rig.sha(), rig.key());
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->sidecars[0].store_sha256, rec->sidecars[0].store_sha256);
    EXPECT_EQ(*found, *rec);
}

TEST(PartImportStore, AnEntryWhoseStoreFileIsMissingOrChangedIsNotFoundAndIsWrittenAgain) {
    StoreRig rig;
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    const fs::path addressed = rig.store / rec->sidecars[0].store_sha256 / rec->sidecars[0].file;
    const std::string bytes = read_file(addressed);
    fs::remove(addressed);
    EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value()) << "a missing store file";
    ASSERT_TRUE(rig.write("t2").has_value());
    EXPECT_EQ(read_file(addressed), bytes) << "written again";
    EXPECT_TRUE(find_entry(rig.store, rig.sha(), rig.key()).has_value());
    write_file(addressed, "not the sidecar");
    EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value()) << "a changed store file";
    ASSERT_TRUE(rig.write("t3").has_value());
    EXPECT_EQ(read_file(addressed), bytes) << "the changed one replaced";
}

TEST(PartImportStore, NoTemporaryIsLeftInAnyStoreFolder) {
    StoreRig rig;
    ASSERT_TRUE(rig.write().has_value());
    int folders = 0;
    for (const auto& d : fs::directory_iterator(rig.store)) {
        ++folders;
        for (const auto& e : fs::directory_iterator(d.path())) {
            EXPECT_NE(e.path().filename().string().rfind(".tmp-", 0), 0u) << e.path().string();
        }
    }
    EXPECT_EQ(folders, 3) << "the source's folder and one folder a sidecar";
}

TEST(PartImportStore, ACrossCheckIsKeptWithTheEntryAndFoundAgain) {
    StoreRig rig;
    rig.objects[0].cross_check_json = R"({"name":"arm left","brep_volume":30,"mesh_volume":27,"difference":0.1,"flagged":true})";
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    const auto j = nlohmann::json::parse(read_file(rec->dir / "import.json"));
    ASSERT_TRUE(j.at("objects")[0].contains("cross_check"));
    EXPECT_EQ(j.at("objects")[0].at("cross_check").at("flagged"), true);
    EXPECT_FALSE(j.at("objects")[1].contains("cross_check"));
    const auto found = find_entry(rig.store, rig.sha(), rig.key());
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, *rec);
    EXPECT_FALSE(found->sidecars[0].cross_check_json.empty());
    EXPECT_EQ(nlohmann::json::parse(found->sidecars[0].cross_check_json).at("difference"), 0.1);
    EXPECT_TRUE(found->sidecars[1].cross_check_json.empty());
}

TEST(PartImportStore, AStorePathThatIsAFileIsStoreIo) {
    StoreRig rig;
    write_file(rig.store, "a file where the store should be");
    const auto rec = rig.write();
    ASSERT_FALSE(rec.has_value());
    EXPECT_EQ(rec.error().code, "store_io");
}

TEST(PartImportStore, StaleTemporariesGoAndFreshOnesStay) {
    test::TempDir tmp{"part_import_stale"};
    const fs::path sha = tmp.path() / std::string(64, 'a');
    write_file(sha / ".tmp-old-key" / "x.spmesh", "x");
    write_file(sha / ".tmp-new-key" / "x.spmesh", "x");
    write_file(sha / "keep" / "x.spmesh", "x");  // not a temporary
    fs::last_write_time(sha / ".tmp-old-key", fs::file_time_type::clock::now() - std::chrono::hours(3));
    EXPECT_EQ(remove_stale_temps(tmp.path(), std::chrono::hours(1)), 1u);
    EXPECT_FALSE(fs::exists(sha / ".tmp-old-key"));
    EXPECT_TRUE(fs::exists(sha / ".tmp-new-key"));
    EXPECT_TRUE(fs::exists(sha / "keep"));
    EXPECT_EQ(remove_stale_temps(tmp.path() / "no-such-store", std::chrono::hours(1)), 0u);
}

// ---- colour --------------------------------------------------------------------------------------------------------------------

TEST(PartImportStore, ImportJsonKeepsEachObjectsColourOrNullBesideTheSidecarAndAFoundEntryReadsItBack) {
    StoreRig rig;
    rig.objects[0].colour = SurfaceColour{{10, 20, 30}, 2};
    const auto rec = rig.write();
    ASSERT_TRUE(rec.has_value()) << rec.error().message;
    const auto j = nlohmann::json::parse(read_file(rec->dir / "import.json"));
    EXPECT_EQ(j.at("objects")[0].at("colour"), (nlohmann::json::array({10, 20, 30})));
    EXPECT_EQ(j.at("objects")[0].at("colours"), 2);
    EXPECT_TRUE(j.at("objects")[1].at("colour").is_null());
    EXPECT_EQ(j.at("objects")[1].at("colours"), 0);
    const auto found = find_entry(rig.store, rig.sha(), rig.key());
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->sidecars[0].colour, (std::optional<SurfaceColour>(SurfaceColour{{10, 20, 30}, 2})));
    EXPECT_FALSE(found->sidecars[1].colour.has_value());
    EXPECT_EQ(*found, *rec);
}

TEST(PartImportStore, AnEntryWrittenBeforeColourIsNotAHitAndIsRewrittenAtTheSameKeyWithTheSameStoreRefs) {
    StoreRig rig;
    rig.objects[0].colour = SurfaceColour{{10, 20, 30}, 1};
    const auto first = rig.write();
    ASSERT_TRUE(first.has_value()) << first.error().message;
    // make it what rev 1 wrote: the version, and no colour fields
    const fs::path file = first->dir / "import.json";
    auto j = nlohmann::json::parse(read_file(file));
    j["import_version"] = 1;
    for (auto& o : j["objects"]) {
        o.erase("colour");
        o.erase("colours");
    }
    write_file(file, j.dump());
    EXPECT_FALSE(find_entry(rig.store, rig.sha(), rig.key()).has_value()) << "an entry with no colour fields would lie: it is not a hit";
    const auto again = rig.write("t2");
    ASSERT_TRUE(again.has_value()) << again.error().message;
    EXPECT_EQ(again->key, first->key);
    ASSERT_EQ(again->sidecars.size(), first->sidecars.size());
    for (std::size_t i = 0; i < again->sidecars.size(); ++i) {
        EXPECT_EQ(again->sidecars[i].store_sha256, first->sidecars[i].store_sha256) << "the sidecar bytes, so their store_refs, do not move";
        EXPECT_EQ(again->sidecars[i].content_hash, first->sidecars[i].content_hash);
    }
    EXPECT_EQ(again->sidecars[0].colour, (std::optional<SurfaceColour>(SurfaceColour{{10, 20, 30}, 1})));
    EXPECT_EQ(nlohmann::json::parse(read_file(file)).at("import_version"), 2);
    EXPECT_TRUE(find_entry(rig.store, rig.sha(), rig.key()).has_value());
}

}  // namespace
