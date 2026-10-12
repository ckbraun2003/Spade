// The editor's terrain edits over the public SdfProgram. Display-free (SL15b);
// Core reviews it (editor plan Task 15, EDT-015).
//
// PIECES. A program is postfix; it is parsed into a tree from its root. Its
// pieces are the subtrees that a chain of PLAIN unions joins at the top, in
// program order. Only a plain union is split: min() is associative and
// commutative, so removing or re-posing one operand of a union chain is exact
// for the others. A smooth union is not associative, so a smooth-union
// subtree is one piece, as are an intersection and a subtraction. A root that
// is not a union is the only piece.
//
// THE EDITS, each returning a new program and never touching its input:
//   add_primitive -- appends a posed primitive and a union, exactly as
//     WorldBuilder would: transform_of() for the pose, the identity pose on
//     transform 0, the union at transform 0 with kind 0 and no parameters.
//   remove_piece -- drops the piece's nodes and the union that joined it,
//     plus only the transforms those nodes used that no other node uses
//     (never transform 0, the identity by convention). So add, then remove,
//     restores the program's bytes -- with one exception: a non-zero
//     material added to a program whose node_materials is empty fills that
//     array with zeros, and the zeros stay after the remove. The meaning is
//     the same and the bytes differ; an all-zero array is not turned back
//     into an empty one, since a world may spell its zeros out. Undo restores
//     the prior program exactly in every case.
//   repose_piece -- gives a piece a new transform. It is refused for a piece
//     whose primitives carry more than one transform, since one pose cannot
//     move them together. A transform another node shares is left alone and
//     the piece gets a new one.
//   set_node_params -- one node's four parameters. An operator takes only
//     what it uses: union, intersect and subtract take [0, 0, 0, 0], and a
//     smooth union takes [k, 0, 0, 0]. Anything else would change the world
//     hash and nothing else, so it is refused (L6).
// An exact no-op (the same transform, the same parameters) returns the input
// unchanged, so it cannot perturb the world hash. node_materials stays
// parallel to the nodes. Every result passes SdfProgram::validate(), and is
// representable in the world file by construction: operator nodes have kind
// 0 at transform 0, and all padding is zero.
//
// docs/design/interface/plans/2026-10-05-editor-design.md §6;
// 2026-10-05-editor-plan.md Task 15; Core's design answers, 2026-10-06.

#pragma once

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace spade::sandbox::editor {

struct SdfPiece {
    uint32_t first = 0;                                // the first node of the piece's subtree
    uint32_t last = 0;                                 // its root node
    uint32_t op = static_cast<uint32_t>(SdfOp::none);  // its root's operator; none for a primitive
};

namespace detail {

[[nodiscard]] inline std::string_view op_word(uint32_t op) {
    switch (static_cast<SdfOp>(op)) {
        case SdfOp::none: return "primitive";
        case SdfOp::union_: return "union";
        case SdfOp::intersect: return "intersect";
        case SdfOp::subtract: return "subtract";
        case SdfOp::smooth_union: return "smooth_union";
    }
    return "operator";
}

[[nodiscard]] inline std::string count_of(std::size_t n, std::string_view one, std::string_view many) {
    return std::to_string(n) + " " + std::string(n == 1 ? one : many);
}

// The tree a postfix program describes.
struct SdfTree {
    std::vector<uint32_t> first;   // per node: the first node of its subtree
    std::vector<int64_t> parent;   // per node: the operator above it, or -1 at the root
    std::vector<uint32_t> left;    // per operator: its deeper operand's root
    std::vector<uint32_t> right;   // per operator: its shallower operand's root
};

[[nodiscard]] inline Result<SdfTree> parse(const SdfProgram& p) {
    if (const Result<uint32_t> ok = p.validate(); !ok) return std::unexpected(ok.error());
    const std::size_t n = p.nodes.size();
    SdfTree t;
    t.first.resize(n);
    t.parent.assign(n, -1);
    t.left.assign(n, 0);
    t.right.assign(n, 0);
    std::vector<uint32_t> stack;
    for (uint32_t i = 0; i < n; ++i) {
        if (p.nodes[i].op == static_cast<uint32_t>(SdfOp::none)) {
            t.first[i] = i;
            stack.push_back(i);
            continue;
        }
        // validate() guarantees two operands here.
        const uint32_t b = stack.back();
        stack.pop_back();
        const uint32_t a = stack.back();
        stack.pop_back();
        t.left[i] = a;
        t.right[i] = b;
        t.parent[a] = i;
        t.parent[b] = i;
        t.first[i] = t.first[a];
        stack.push_back(i);
    }
    return t;
}

// Iterative, so a long union chain (one per composed asset) cannot run the stack out.
[[nodiscard]] inline std::vector<SdfPiece> pieces_of(const SdfProgram& p, const SdfTree& t) {
    std::vector<SdfPiece> out;
    if (p.nodes.empty()) return out;
    std::vector<uint32_t> todo{static_cast<uint32_t>(p.nodes.size() - 1)};
    while (!todo.empty()) {
        const uint32_t n = todo.back();
        todo.pop_back();
        if (p.nodes[n].op == static_cast<uint32_t>(SdfOp::union_)) {
            todo.push_back(t.right[n]);
            todo.push_back(t.left[n]);  // popped first: program order
            continue;
        }
        out.push_back(SdfPiece{t.first[n], n, p.nodes[n].op});
    }
    return out;
}

// Bit for bit: -0 and 0 differ, as they do in the world hash.
[[nodiscard]] inline bool same_transform(const SdfTransform& a, const SdfTransform& b) {
    return std::memcmp(&a, &b, sizeof(SdfTransform)) == 0;
}

// `p` without the nodes marked in `gone`, and without each transform in
// `candidates` that no remaining node uses (never transform 0). The remaining
// transforms keep their order and the nodes are re-pointed at them.
[[nodiscard]] inline SdfProgram without(const SdfProgram& p, const std::vector<bool>& gone,
                                        const std::vector<uint32_t>& candidates) {
    std::vector<bool> used(p.transforms.size(), false);
    for (std::size_t i = 0; i < p.nodes.size(); ++i) {
        if (!gone[i]) used[p.nodes[i].transform] = true;
    }
    std::vector<bool> drop(p.transforms.size(), false);
    for (const uint32_t c : candidates) {
        if (c != 0 && c < drop.size() && !used[c]) drop[c] = true;
    }
    SdfProgram out;
    std::vector<uint32_t> remap(p.transforms.size(), 0);
    for (std::size_t t = 0; t < p.transforms.size(); ++t) {
        if (drop[t]) continue;
        remap[t] = static_cast<uint32_t>(out.transforms.size());
        out.transforms.push_back(p.transforms[t]);
    }
    for (std::size_t i = 0; i < p.nodes.size(); ++i) {
        if (gone[i]) continue;
        SdfNode node = p.nodes[i];
        node.transform = remap[node.transform];
        out.nodes.push_back(node);
        if (!p.node_materials.empty()) out.node_materials.push_back(p.node_materials[i]);
    }
    return out;
}

[[nodiscard]] inline Result<SdfProgram> checked(SdfProgram p) {
    if (const Result<uint32_t> ok = p.validate(); !ok) return std::unexpected(ok.error());
    return p;
}

[[nodiscard]] inline Error no_piece(std::string_view what, std::size_t k, std::size_t count) {
    return Error{Code::invalid_argument, std::string(what) + " terrain piece " + std::to_string(k) +
                                             ": the program has " + count_of(count, "piece", "pieces") +
                                             ", numbered from 0"};
}

}  // namespace detail

// The program's pieces, in program order; none for an empty program.
[[nodiscard]] inline Result<std::vector<SdfPiece>> union_pieces(const SdfProgram& program) {
    const Result<detail::SdfTree> tree = detail::parse(program);
    if (!tree) return std::unexpected(tree.error());
    return detail::pieces_of(program, *tree);
}

// A primitive at `pose`, unioned onto the program (or the whole program, if
// it was empty). `material` indexes the world's palette; 0 is the default.
[[nodiscard]] inline Result<SdfProgram> add_primitive(const SdfProgram& program, SdfPrim kind, glm::vec4 params,
                                                      const SdfPose& pose, uint32_t material = 0) {
    const Result<SdfTransform> t = transform_of(pose);
    if (!t) return std::unexpected(t.error());
    SdfProgram out = program;
    if (out.transforms.empty()) out.transforms.push_back(SdfTransform{});
    uint32_t index = 0;
    if (!detail::same_transform(*t, out.transforms[0])) {
        index = static_cast<uint32_t>(out.transforms.size());
        out.transforms.push_back(*t);
    }
    const bool joins = !out.nodes.empty();
    const bool materials = !out.node_materials.empty() || material != 0;
    if (materials && out.node_materials.empty()) out.node_materials.assign(out.nodes.size(), 0u);

    SdfNode prim{};
    prim.kind = static_cast<uint32_t>(kind);
    prim.op = static_cast<uint32_t>(SdfOp::none);
    prim.transform = index;
    prim.params = params;
    out.nodes.push_back(prim);
    if (materials) out.node_materials.push_back(material);
    if (joins) {
        SdfNode join{};
        join.op = static_cast<uint32_t>(SdfOp::union_);
        out.nodes.push_back(join);
        if (materials) out.node_materials.push_back(0u);
    }
    return detail::checked(std::move(out));
}

// The program without piece `k` and the union that joined it.
[[nodiscard]] inline Result<SdfProgram> remove_piece(const SdfProgram& program, std::size_t k) {
    const Result<detail::SdfTree> tree = detail::parse(program);
    if (!tree) return std::unexpected(tree.error());
    const std::vector<SdfPiece> pieces = detail::pieces_of(program, *tree);
    if (k >= pieces.size()) return std::unexpected(detail::no_piece("remove", k, pieces.size()));
    if (pieces.size() == 1) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "remove terrain piece 0: it is the program's only piece, and a world keeps at "
                                     "least one; add another piece first, or edit this one"});
    }
    const SdfPiece& piece = pieces[k];
    std::vector<bool> gone(program.nodes.size(), false);
    std::vector<uint32_t> candidates;
    for (uint32_t i = piece.first; i <= piece.last; ++i) {
        gone[i] = true;
        candidates.push_back(program.nodes[i].transform);
    }
    // More than one piece, so the root is a union and this piece has one above it.
    const auto join = static_cast<std::size_t>(tree->parent[piece.last]);
    gone[join] = true;
    return detail::checked(detail::without(program, gone, candidates));
}

// Piece `k` at `pose`.
[[nodiscard]] inline Result<SdfProgram> repose_piece(const SdfProgram& program, std::size_t k, const SdfPose& pose) {
    const Result<detail::SdfTree> tree = detail::parse(program);
    if (!tree) return std::unexpected(tree.error());
    const std::vector<SdfPiece> pieces = detail::pieces_of(program, *tree);
    if (k >= pieces.size()) return std::unexpected(detail::no_piece("repose", k, pieces.size()));
    const SdfPiece& piece = pieces[k];
    const auto primitive = [&](std::size_t i) { return program.nodes[i].op == static_cast<uint32_t>(SdfOp::none); };

    std::vector<uint32_t> poses;
    for (uint32_t i = piece.first; i <= piece.last; ++i) {
        if (!primitive(i)) continue;
        const uint32_t t = program.nodes[i].transform;
        if (std::find(poses.begin(), poses.end(), t) == poses.end()) poses.push_back(t);
    }
    if (poses.size() > 1) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "repose terrain piece " + std::to_string(k) + ": it is a " +
                                         std::string(detail::op_word(piece.op)) + " of primitives with " +
                                         std::to_string(poses.size()) +
                                         " poses of their own, and one pose cannot move them together without "
                                         "changing its shape; edit those primitives, or replace the piece"});
    }
    const Result<SdfTransform> t = transform_of(pose);
    if (!t) return std::unexpected(t.error());
    const uint32_t old_index = poses[0];
    if (detail::same_transform(program.transforms[old_index], *t)) return program;  // exact no-op

    bool shared = false;
    for (std::size_t i = 0; i < program.nodes.size(); ++i) {
        const bool inside = i >= piece.first && i <= piece.last && primitive(i);
        if (!inside && program.nodes[i].transform == old_index) shared = true;
    }
    SdfProgram out = program;
    uint32_t new_index = 0;
    if (detail::same_transform(*t, SdfTransform{}) && detail::same_transform(out.transforms[0], SdfTransform{})) {
        new_index = 0;  // the identity, as the builder does it
    } else if (!shared && old_index != 0) {
        out.transforms[old_index] = *t;  // the piece's own transform, changed in place
        return detail::checked(std::move(out));
    } else {
        new_index = static_cast<uint32_t>(out.transforms.size());
        out.transforms.push_back(*t);
    }
    for (uint32_t i = piece.first; i <= piece.last; ++i) {
        if (primitive(i) && out.nodes[i].transform == old_index) out.nodes[i].transform = new_index;
    }
    return detail::checked(detail::without(out, std::vector<bool>(out.nodes.size(), false), {old_index}));
}

// Node `node`'s four parameters (sdf.hpp lists each kind's packing).
[[nodiscard]] inline Result<SdfProgram> set_node_params(const SdfProgram& program, std::size_t node, glm::vec4 params) {
    if (node >= program.nodes.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "set the parameters of terrain node " + std::to_string(node) +
                                         ": the program has " + detail::count_of(program.nodes.size(), "node", "nodes") +
                                         ", numbered from 0"});
    }
    if (std::memcmp(&program.nodes[node].params, &params, sizeof(glm::vec4)) == 0) return program;  // exact no-op
    const uint32_t op = program.nodes[node].op;
    if (op != static_cast<uint32_t>(SdfOp::none)) {
        const bool blend = op == static_cast<uint32_t>(SdfOp::smooth_union);
        // By bits: -0 is not 0 to the writer or the hash. A smooth union's k
        // is the user's own value, -0 included (k <= 0 degenerates to min).
        const auto set = [](float v) { return std::bit_cast<uint32_t>(v) != 0u; };
        const bool unused_set = (!blend && set(params.x)) || set(params.y) || set(params.z) || set(params.w);
        if (unused_set) {
            return std::unexpected(Error{
                Code::invalid_argument,
                "set the parameters of terrain node " + std::to_string(node) + ": it is a " +
                    std::string(detail::op_word(op)) + ", which takes " +
                    (blend ? "only its blend radius [k, 0, 0, 0]" : "no parameters [0, 0, 0, 0]") +
                    "; any other value would change the world's hash and nothing else"});
        }
    }
    SdfProgram out = program;
    out.nodes[node].params = params;
    return detail::checked(std::move(out));
}

}  // namespace spade::sandbox::editor
