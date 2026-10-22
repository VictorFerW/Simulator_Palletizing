//! Padrões de paletização: from the product, the robot pick (products side by
//! side) and the pallet, generates the most used layer patterns and lets the user
//! apply one to a TDP de paletização, which the robot then follows.
//!
//! Each position is one product. The cabeçote carries up to two products side by
//! side (one per clamp): a pick places both together on two neighbouring positions
//! ("par", both clamps open) or one at a time ("simples": one clamp opens, the robot
//! lifts, turns and places the other, possibly on the next layer). When one
//! product is left for the pallet the mesa sends a single one.
//!
//! Patterns:
//! - Coluna: every unit in the same orientation, layers identical;
//! - Coluna cruzada: odd layers in one orientation, even layers turned 90°;
//! - Amarração (tijolo): a block lengthwise and a block crosswise, even layers
//!   mirrored so the joints do not line up;
//! - Catavento (pinwheel): four blocks turning around the centre, even layers mirrored.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_egui::egui;
use serde::{Deserialize, Serialize};

/// A layer pattern stored on a TDP de paletização.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Pattern {
    pub name: String,
    /// Product [length X, height, width Z] (m).
    pub unit: [f32; 3],
    pub layers: u32,
    /// Placements of odd (1st, 3rd...) and even layers: [x, z, yaw] in the pallet
    /// frame (origin at the pallet centre, yaw 0 = product length along the pallet X).
    pub odd: Vec<[f32; 3]>,
    pub even: Vec<[f32; 3]>,
    /// Products carried per pick (1 or 2) and their spacing in the cabeçote.
    #[serde(default = "one")]
    pub per_pick: u32,
    #[serde(default)]
    pub pick_gap: f32,
    #[serde(default)]
    pub pick_in_line: bool,
    /// Picks of the robot: each pick is a list of drops, each drop the slots it fills.
    #[serde(default)]
    pub picks: Vec<Vec<Vec<u32>>>,
    /// Groups the mesa forms, repeated (receita 10x1: [[2, 2], [3], [3]] = 4 fardos placed
    /// 2 + 2 at the two ends, then 3 and 3). Empty: pairs and singles of `per_pick`.
    #[serde(default)]
    pub groups: Vec<Vec<u32>>,
}

fn one() -> u32 {
    1
}

impl Pattern {
    /// Slots in placing order.
    pub fn sequence(&self) -> Vec<u32> {
        if self.picks.is_empty() {
            (0..self.capacity()).collect()
        } else {
            self.picks.iter().flatten().flatten().copied().collect()
        }
    }

    pub fn capacity(&self) -> u32 {
        (0..self.layers).map(|l| self.layer(l).len() as u32).sum()
    }

    pub fn layer(&self, layer: u32) -> &[[f32; 3]] {
        if layer.is_multiple_of(2) { &self.odd } else { &self.even }
    }

    /// Two positions of a layer the cabeçote can fill in one drop: same orientation,
    /// next to each other across the product width (along the length when in line).
    pub fn pairable(&self, a: &[f32; 3], b: &[f32; 3]) -> bool {
        if self.per_pick < 2 {
            return false;
        }
        let turn = (a[2] - b[2]).rem_euclid(std::f32::consts::PI);
        if turn > 0.05 && turn < std::f32::consts::PI - 0.05 {
            return false;
        }
        let along = Vec2::new(a[2].cos(), -a[2].sin());
        let d = Vec2::new(b[0] - a[0], b[1] - a[1]);
        let (axial, lateral) = (d.dot(along).abs(), d.perp_dot(along).abs());
        let (spacing, offset, size) = if self.pick_in_line {
            (axial, lateral, self.unit[0])
        } else {
            (lateral, axial, self.unit[2])
        };
        offset < 0.03 && spacing > size - 0.01 && spacing < size + self.pick_gap.max(0.0) + 0.05
    }
}

/// Greedy maximum matching of the pairable positions in `set` (positions of one layer).
fn matching(p: &Pattern, places: &[[f32; 3]], set: &[usize]) -> Vec<(usize, usize)> {
    let mut used = vec![false; set.len()];
    let mut pairs = Vec::new();
    for i in 0..set.len() {
        if used[i] {
            continue;
        }
        if let Some(j) = ((i + 1)..set.len()).find(|&j| !used[j] && p.pairable(&places[set[i]], &places[set[j]])) {
            used[i] = true;
            used[j] = true;
            pairs.push((set[i], set[j]));
        }
    }
    pairs
}

/// Straight runs of `k` side-by-side positions among `left` (k = 1: each position).
fn chains(p: &Pattern, places: &[[f32; 3]], left: &[usize], k: usize) -> Vec<Vec<usize>> {
    if k <= 1 {
        return left.iter().map(|&i| vec![i]).collect();
    }
    let mut out = Vec::new();
    for &a in left {
        for &b in left {
            if a == b || !p.pairable(&places[a], &places[b]) {
                continue;
            }
            let step = Vec2::new(places[b][0] - places[a][0], places[b][1] - places[a][1]);
            let mut run = vec![a, b];
            while run.len() < k {
                let last = places[*run.last().unwrap()];
                let next = left.iter().copied().find(|&c| {
                    !run.contains(&c)
                        && p.pairable(&last, &places[c])
                        && (Vec2::new(places[c][0] - last[0], places[c][1] - last[1]) - step).length() < 0.03
                });
                match next {
                    Some(c) => run.push(c),
                    None => break,
                }
            }
            if run.len() == k {
                out.push(run);
            }
        }
    }
    out
}

/// Positions of `left` with no side-by-side neighbour left (they could only be placed alone).
fn isolated(p: &Pattern, places: &[[f32; 3]], left: &[usize]) -> usize {
    left.iter().filter(|&&i| !left.iter().any(|&j| j != i && p.pairable(&places[i], &places[j]))).count()
}

/// Pick sequence for the groups of the mesa (`p.groups`, repeated): each drop is a
/// straight run of side-by-side positions, chosen at the ends of the rows so the rest
/// still forms runs (the 2 + 2 of a group of 4 go to the two ends). A layer that does
/// not fit the groups takes the rest one by one.
fn plan_groups(p: &Pattern) -> Vec<Vec<Vec<u32>>> {
    let mut picks = Vec::new();
    let mut cycle = 0;
    let mut offset = 0u32;
    for layer in 0..p.layers {
        let places = p.layer(layer);
        let mut left: Vec<usize> = (0..places.len()).collect();
        while !left.is_empty() {
            let spec = &p.groups[cycle % p.groups.len()];
            cycle += 1;
            let mut trial = left.clone();
            let mut drops: Vec<Vec<u32>> = Vec::new();
            for &k in spec {
                let best = chains(p, places, &trial, k as usize).into_iter().min_by_key(|run| {
                    let rest: Vec<usize> = trial.iter().copied().filter(|i| !run.contains(i)).collect();
                    let degree: usize = run
                        .iter()
                        .map(|&i| trial.iter().filter(|&&j| j != i && p.pairable(&places[i], &places[j])).count())
                        .sum();
                    (isolated(p, places, &rest), degree)
                });
                match best {
                    Some(run) => {
                        trial.retain(|i| !run.contains(i));
                        drops.push(run.iter().map(|&i| offset + i as u32).collect());
                    }
                    None => {
                        drops.clear();
                        break;
                    }
                }
            }
            if drops.is_empty() {
                // Does not fit the group: the rest of the layer one at a time.
                let n = (spec.iter().sum::<u32>() as usize).min(left.len());
                drops = left.drain(..n).map(|i| vec![offset + i as u32]).collect();
            } else {
                left = trial;
            }
            picks.push(drops);
        }
        offset += places.len() as u32;
    }
    picks
}

/// Pick sequence of the robot (see the module comment): on each layer, first two
/// positions that cannot pair (two simple drops), then the pairs; a position left
/// alone goes with the first simple drop of the next layer; the last one alone is
/// a single-product pick.
pub fn plan_picks(p: &Pattern) -> Vec<Vec<Vec<u32>>> {
    if !p.groups.is_empty() {
        return plan_groups(p);
    }
    let mut picks: Vec<Vec<Vec<u32>>> = Vec::new();
    let mut open: Option<Vec<Vec<u32>>> = None;
    let mut offset = 0u32;
    for layer in 0..p.layers {
        let places = p.layer(layer);
        let mut left: Vec<usize> = (0..places.len()).collect();
        let slot = |i: usize| offset + i as u32;
        // Best position to take alone: keeps the most pairs among the others.
        let best_single = |left: &[usize]| -> usize {
            let score = |k: usize| {
                let rest: Vec<usize> = left.iter().copied().filter(|&i| i != k).collect();
                matching(p, places, &rest).len()
            };
            let mut best = left[0];
            for &k in left {
                if score(k) > score(best) {
                    best = k;
                }
            }
            best
        };
        while !left.is_empty() {
            if p.per_pick < 2 {
                let i = left.remove(0);
                picks.push(vec![vec![slot(i)]]);
                continue;
            }
            if let Some(mut pick) = open.take() {
                // Second product of a pick started on the layer below.
                let i = best_single(&left);
                left.retain(|&k| k != i);
                pick.push(vec![slot(i)]);
                picks.push(pick);
                continue;
            }
            let pairs = matching(p, places, &left);
            let paired: Vec<usize> = pairs.iter().flat_map(|&(a, b)| [a, b]).collect();
            let alone: Vec<usize> = left.iter().copied().filter(|i| !paired.contains(i)).collect();
            if alone.len() >= 2 {
                picks.push(vec![vec![slot(alone[0])], vec![slot(alone[1])]]);
                left.retain(|&k| k != alone[0] && k != alone[1]);
            } else if let Some(&(a, b)) = pairs.first() {
                picks.push(vec![vec![slot(a), slot(b)]]);
                left.retain(|&k| k != a && k != b);
            } else {
                let i = left.remove(0);
                open = Some(vec![vec![slot(i)]]);
            }
        }
        offset += places.len() as u32;
    }
    if let Some(pick) = open {
        picks.push(pick);
    }
    picks
}

impl Pattern {
    /// Placement `index` in the pallet frame (centre, yaw) for units of height `unit_h`
    /// on a pallet of height `pallet_h`; `None` when the pallet is complete.
    pub fn slot(&self, pallet_h: f32, unit_h: f32, index: u32) -> Option<(Vec3, f32)> {
        let mut remaining = index as usize;
        for layer in 0..self.layers {
            let places = if layer % 2 == 0 { &self.odd } else { &self.even };
            if remaining < places.len() {
                let [x, z, yaw] = places[remaining];
                let y = pallet_h / 2.0 + layer as f32 * unit_h + unit_h / 2.0;
                return Some((Vec3::new(x, y, z), yaw));
            }
            remaining -= places.len();
        }
        None
    }
}

/// Inputs of the pattern screen (mm, kg).
#[derive(Clone, Debug, PartialEq)]
pub struct PatternInput {
    pub product: [f32; 3],
    pub product_kg: f32,
    pub per_pick: u32,
    pub pick_gap: f32,
    /// Products of a pick in line along their length (else side by side across their width).
    pub pick_in_line: bool,
    /// Mesa 3 estágios receita 10x1: groups of 4 (placed 2 + 2) and 3, 3.
    pub recipe_10x1: bool,
    pub pallet: [f32; 3],
    pub overhang: f32,
    pub gap: f32,
    pub max_height: f32,
    pub max_kg: f32,
    pub layers: u32,
}

impl Default for PatternInput {
    fn default() -> Self {
        Self {
            product: [400.0, 300.0, 250.0],
            product_kg: 12.0,
            per_pick: 2,
            pick_gap: 20.0,
            pick_in_line: false,
            recipe_10x1: false,
            pallet: [1000.0, 1200.0, 144.0],
            overhang: 0.0,
            gap: 0.0,
            max_height: 1900.0,
            max_kg: 1500.0,
            layers: 0,
        }
    }
}

impl PatternInput {
    /// Product [length, height, width] (mm): one position of the pattern.
    pub fn unit(&self) -> [f32; 3] {
        let [l, w, h] = self.product;
        [l, h, w]
    }

    pub fn unit_kg(&self) -> f32 {
        self.product_kg
    }
}

/// Most used pallet sizes (length × width × height, mm).
pub const PALLETS: &[(&str, [f32; 3])] = &[
    ("PBR 1000 × 1200 (padrão Brasil)", [1000.0, 1200.0, 144.0]),
    ("Europeu 800 × 1200 (EUR 1)", [800.0, 1200.0, 144.0]),
    ("Industrial 1200 × 1000 (EUR 2)", [1200.0, 1000.0, 144.0]),
    ("Americano 1016 × 1219 (40 × 48\")", [1016.0, 1219.0, 140.0]),
    ("Quadrado 1100 × 1100", [1100.0, 1100.0, 144.0]),
    ("Asiático 1100 × 1300", [1100.0, 1300.0, 144.0]),
    ("Químico 1140 × 1140", [1140.0, 1140.0, 156.0]),
];

/// A generated option with its numbers.
#[derive(Clone, Debug)]
pub struct Candidate {
    pub pattern: Pattern,
    pub per_layer: usize,
    pub total: usize,
    /// Pallet area covered by one layer (%).
    pub area_use: f32,
    /// Largest overhang beyond the pallet edge (mm).
    pub overhang: f32,
    pub height: f32,
    pub kg: f32,
    pub interlocked: bool,
    /// Every pick matches the groups of the mesa (receita 10x1: 4 = 2 + 2, 3, 3).
    pub fits_groups: bool,
    /// Layers limited by: altura, peso or the requested count.
    pub limit: &'static str,
}

/// Units of `size` (+ `gap` between them) that fit in `len`.
fn fit(len: f32, size: f32, gap: f32) -> usize {
    if size <= 0.0 {
        return 0;
    }
    (((len + gap) / (size + gap)) + 1e-4).floor().max(0.0) as usize
}

/// Placements of a block of `cols × rows` units starting at the corner (x0, z0),
/// lengthwise (`a` along X) or turned (`a` along Z). Coordinates are unit centres.
fn block(x0: f32, z0: f32, cols: usize, rows: usize, rotated: bool, (a, b, g): (f32, f32, f32)) -> Vec<[f32; 3]> {
    let (sx, sz) = if rotated { (b, a) } else { (a, b) };
    let yaw = if rotated { FRAC_PI_2 } else { 0.0 };
    let mut out = Vec::new();
    for r in 0..rows {
        for c in 0..cols {
            out.push([x0 + sx / 2.0 + c as f32 * (sx + g), z0 + sz / 2.0 + r as f32 * (sz + g), yaw]);
        }
    }
    out
}

fn footprint(p: &[f32; 3], (a, b): (f32, f32)) -> (f32, f32) {
    if p[2].abs() > 0.1 { (b, a) } else { (a, b) }
}

/// Centres the layer on the pallet and orders the placements row by row.
fn centre(mut places: Vec<[f32; 3]>, (a, b): (f32, f32)) -> Vec<[f32; 3]> {
    let (mut lo, mut hi) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
    for p in &places {
        let (sx, sz) = footprint(p, (a, b));
        lo = lo.min(Vec2::new(p[0] - sx / 2.0, p[1] - sz / 2.0));
        hi = hi.max(Vec2::new(p[0] + sx / 2.0, p[1] + sz / 2.0));
    }
    let c = (lo + hi) / 2.0;
    for p in &mut places {
        p[0] -= c.x;
        p[1] -= c.y;
    }
    places.sort_by(|p, q| (p[1] * 100.0).round().total_cmp(&(q[1] * 100.0).round()).then(p[0].total_cmp(&q[0])));
    places
}

fn mirror_x(places: &[[f32; 3]]) -> Vec<[f32; 3]> {
    places.iter().map(|p| [-p[0], p[1], p[2]]).collect()
}

/// Best two-block layer: lengthwise block on one side, crosswise on the other,
/// split along X (or along Z when `along_z`).
fn two_blocks(w: f32, d: f32, (a, b, g): (f32, f32, f32), along_z: bool) -> Vec<[f32; 3]> {
    let (span, other) = if along_z { (d, w) } else { (w, d) };
    let mut best: Vec<[f32; 3]> = Vec::new();
    let mut best_mixed = false;
    // Block 1: `k` units across the split direction, unit side `s1` along it.
    for rotated_first in [false, true] {
        let (s1, s2) = match (along_z, rotated_first) {
            (false, false) | (true, true) => (a, b),
            _ => (b, a),
        };
        for k in 0..=fit(span, s1, g) {
            let used = if k == 0 { 0.0 } else { k as f32 * (s1 + g) };
            let rest = span - used;
            let k2 = fit(rest, s2, g);
            let (rows1, rows2) = (fit(other, s2, g), fit(other, s1, g));
            let mut places = Vec::new();
            if along_z {
                places.extend(block(0.0, 0.0, rows1, k, rotated_first, (a, b, g)));
                places.extend(block(0.0, used, rows2, k2, !rotated_first, (a, b, g)));
            } else {
                places.extend(block(0.0, 0.0, k, rows1, rotated_first, (a, b, g)));
                places.extend(block(used, 0.0, k2, rows2, !rotated_first, (a, b, g)));
            }
            let mixed = k > 0 && k2 > 0 && rows1 > 0 && rows2 > 0;
            if places.len() > best.len() || (places.len() == best.len() && mixed && !best_mixed) {
                best = places;
                best_mixed = mixed;
            }
        }
    }
    if best_mixed { best } else { Vec::new() }
}

/// Best pinwheel: four blocks around the centre (top-left and bottom-right
/// lengthwise, top-right and bottom-left crosswise), each one touching a corner.
fn pinwheel(w: f32, d: f32, (a, b, g): (f32, f32, f32)) -> Vec<[f32; 3]> {
    let pitch = |n: usize, s: f32| if n == 0 { 0.0 } else { n as f32 * (s + g) - g };
    let mut best: Vec<[f32; 3]> = Vec::new();
    let max_n = 6;
    for i in 1..=fit(w, a, g).min(max_n) {
        for j in 1..=fit(d, b, g).min(max_n) {
            let (x1, z1) = (pitch(i, a), pitch(j, b));
            // Top-right, crosswise, against the right edge.
            for p in 1..=fit(w - x1 - g, b, g).min(max_n) {
                let x2 = w - pitch(p, b);
                let q = fit(d, a, g).min(max_n);
                for q in 1..=q {
                    let z2 = pitch(q, a);
                    // Bottom-right, lengthwise.
                    for r in 1..=fit(w, a, g).min(max_n) {
                        let x3 = w - pitch(r, a);
                        let s = fit(d - z2 - g, b, g).min(max_n);
                        if s == 0 {
                            continue;
                        }
                        let z3 = d - pitch(s, b);
                        // Bottom-left, crosswise.
                        let t = fit(x3 - g, b, g).min(max_n);
                        let u = fit(d - z1 - g, a, g).min(max_n);
                        if t == 0 || u == 0 {
                            continue;
                        }
                        let (x4, z4) = (pitch(t, b), d - pitch(u, a));
                        // No overlaps between neighbours and across the centre.
                        let sep = |ax0: f32, ax1: f32, az0: f32, az1: f32, bx0: f32, bx1: f32, bz0: f32, bz1: f32| {
                            ax1 + g <= bx0 + 1e-4 || bx1 + g <= ax0 + 1e-4 || az1 + g <= bz0 + 1e-4 || bz1 + g <= az0 + 1e-4
                        };
                        let blocks = [(0.0, x1, 0.0, z1), (x2, w, 0.0, z2), (x3, w, z3, d), (0.0, x4, z4, d)];
                        let ok = (0..4).all(|m| {
                            ((m + 1)..4).all(|n| {
                                let (p0, q0) = (blocks[m], blocks[n]);
                                sep(p0.0, p0.1, p0.2, p0.3, q0.0, q0.1, q0.2, q0.3)
                            })
                        });
                        if !ok {
                            continue;
                        }
                        let count = i * j + p * q + r * s + t * u;
                        if count > best.len() {
                            let mut places = block(0.0, 0.0, i, j, false, (a, b, g));
                            places.extend(block(x2, 0.0, p, q, true, (a, b, g)));
                            places.extend(block(x3, z3, r, s, false, (a, b, g)));
                            places.extend(block(0.0, z4, t, u, true, (a, b, g)));
                            best = places;
                        }
                    }
                }
            }
        }
    }
    best
}

type Layer = Vec<[f32; 3]>;
type LayerOption = (String, Layer, Layer, bool);

/// Generates the options for the inputs, best first.
pub fn generate(input: &PatternInput) -> Vec<Candidate> {
    let mm = |v: f32| v / 1000.0;
    let unit = input.unit();
    let (a, b, g) = (mm(unit[0]), mm(unit[2]), mm(input.gap));
    let (pw, pd) = (mm(input.pallet[0]), mm(input.pallet[1]));
    let (w, d) = (pw + 2.0 * mm(input.overhang), pd + 2.0 * mm(input.overhang));
    if a <= 0.0 || b <= 0.0 || w <= 0.0 || d <= 0.0 {
        return Vec::new();
    }
    let ab = (a, b, g);
    // (name, odd layer, even layer, interlocked)
    let mut layers_options: Vec<LayerOption> = Vec::new();

    let col_a = block(0.0, 0.0, fit(w, a, g), fit(d, b, g), false, ab);
    let col_b = block(0.0, 0.0, fit(w, b, g), fit(d, a, g), true, ab);
    // Both orientations of the column (the one with fewer may be the recipe, e.g. 10x1 in 5 x 2).
    for (name, column) in [("Coluna", &col_a), ("Coluna (produto atravessado)", &col_b)] {
        if !column.is_empty() && col_a.len() != col_b.len() {
            let column = centre(column.clone(), (a, b));
            layers_options.push((name.into(), column.clone(), column, false));
        }
    }
    if !col_a.is_empty() && col_a.len() == col_b.len() {
        let column = centre(col_a.clone(), (a, b));
        layers_options.push(("Coluna".into(), column.clone(), column, false));
    }
    if !col_a.is_empty() && col_a.len() == col_b.len() {
        layers_options.push(("Coluna cruzada".into(), centre(col_a, (a, b)), centre(col_b, (a, b)), true));
    }
    for along_z in [false, true] {
        let layer = two_blocks(w, d, ab, along_z);
        if !layer.is_empty() {
            let odd = centre(layer, (a, b));
            let even = centre(if along_z { odd.iter().map(|p| [p[0], -p[1], p[2]]).collect() } else { mirror_x(&odd) }, (a, b));
            let name = if along_z { "Amarração (tijolo) na largura" } else { "Amarração (tijolo) no comprimento" };
            layers_options.push((name.into(), odd, even, true));
        }
    }
    let wheel = pinwheel(w, d, ab);
    if !wheel.is_empty() {
        let odd = centre(wheel, (a, b));
        let even = centre(mirror_x(&odd), (a, b));
        layers_options.push(("Catavento (pinwheel)".into(), odd, even, true));
    }

    let unit_h = mm(unit[1]);
    let mut out: Vec<Candidate> = layers_options
        .into_iter()
        .filter(|(_, odd, even, _)| odd.len() == even.len())
        .map(|(name, odd, even, interlocked)| {
            let per_layer = odd.len();
            let by_height = ((mm(input.max_height) - mm(input.pallet[2])) / unit_h + 1e-4).floor().max(0.0) as u32;
            let layer_kg = per_layer as f32 * input.unit_kg();
            let by_weight = if layer_kg > 0.0 { (input.max_kg / layer_kg + 1e-4).floor() as u32 } else { u32::MAX };
            let (mut layers, mut limit) = if by_weight < by_height { (by_weight, "peso") } else { (by_height, "altura") };
            if input.layers > 0 && input.layers <= layers {
                layers = input.layers;
                limit = "camadas pedidas";
            }
            let overhang = odd
                .iter()
                .map(|p| {
                    let (sx, sz) = footprint(p, (a, b));
                    ((p[0].abs() + sx / 2.0 - pw / 2.0).max(p[1].abs() + sz / 2.0 - pd / 2.0)).max(0.0)
                })
                .fold(0.0, f32::max);
            Candidate {
                per_layer,
                total: per_layer * layers as usize,
                area_use: 100.0 * per_layer as f32 * a * b / (pw * pd),
                overhang: overhang * 1000.0,
                height: input.pallet[2] + layers as f32 * unit[1],
                kg: layers as f32 * layer_kg,
                interlocked,
                fits_groups: true,
                limit,
                pattern: {
                    let mut pattern = Pattern {
                        name,
                        unit: [a, unit_h, b],
                        layers,
                        odd,
                        even,
                        per_pick: if input.recipe_10x1 { 4 } else { input.per_pick.clamp(1, 2) },
                        pick_gap: mm(input.pick_gap),
                        pick_in_line: input.pick_in_line,
                        picks: Vec::new(),
                        groups: if input.recipe_10x1 { vec![vec![2, 2], vec![3], vec![3]] } else { Vec::new() },
                    };
                    pattern.picks = plan_picks(&pattern);
                    pattern
                },
            }
        })
        .collect();
    for c in &mut out {
        let g = &c.pattern.groups;
        c.fits_groups = g.is_empty()
            || c.pattern.picks.iter().enumerate().all(|(i, pick)| {
                let sizes: Vec<u32> = pick.iter().map(|d| d.len() as u32).collect();
                sizes == g[i % g.len()]
            });
    }
    out.sort_by(|x, y| {
        y.fits_groups.cmp(&x.fits_groups).then(y.total.cmp(&x.total)).then(y.interlocked.cmp(&x.interlocked))
    });
    out
}

/// State of the pattern window.
#[derive(Default)]
pub struct PatternUi {
    pub open: bool,
    pub input: PatternInput,
    /// Inputs of the last generation (regenerated when they change).
    generated_for: Option<PatternInput>,
    candidates: Vec<Candidate>,
    selected: usize,
    /// Name of the TDP de paletização to apply to.
    pub target: String,
    pub message: String,
    /// PDT_PATTERN opens the window at start (automated screenshots);
    /// PDT_PATTERN=apply also applies the best option to the first TDP.
    pub env_checked: bool,
    pub auto_apply: bool,
    close_after: bool,
    /// Emitters / mesas checked to receive the product size.
    size_targets: Vec<String>,
    /// Names and box size (m, [length, height, width]) to apply, taken by the caller.
    pub size_request: Option<(Vec<String>, [f32; 3])>,
}

fn mm_field(ui: &mut egui::Ui, label: &str, value: &mut f32, max: f32, suffix: &str) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(5.0).range(0.0..=max).suffix(suffix));
    ui.end_row();
}

/// Draws one layer seen from above, units numbered in placing order.
fn draw_layer(ui: &mut egui::Ui, title: &str, places: &[[f32; 3]], labels: &[String], unit: [f32; 3], pallet: (f32, f32), size: f32) {
    ui.vertical(|ui| {
        ui.small(title);
        let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let (mut ext_x, mut ext_z) = (pallet.0, pallet.1);
        for p in places {
            let (sx, sz) = footprint(p, (unit[0], unit[2]));
            ext_x = ext_x.max(2.0 * (p[0].abs() + sx / 2.0));
            ext_z = ext_z.max(2.0 * (p[1].abs() + sz / 2.0));
        }
        let scale = (size - 10.0) / ext_x.max(ext_z);
        let c = rect.center();
        // Pallet X (TDP travel) to the right, Z down.
        let to = |x: f32, z: f32| egui::pos2(c.x + x * scale, c.y + z * scale);
        let wood = egui::Color32::from_rgb(170, 125, 75);
        painter.rect_filled(
            egui::Rect::from_two_pos(to(-pallet.0 / 2.0, -pallet.1 / 2.0), to(pallet.0 / 2.0, pallet.1 / 2.0)),
            2.0,
            wood,
        );
        for (i, p) in places.iter().enumerate() {
            let (sx, sz) = footprint(p, (unit[0], unit[2]));
            let r = egui::Rect::from_two_pos(to(p[0] - sx / 2.0, p[1] - sz / 2.0), to(p[0] + sx / 2.0, p[1] + sz / 2.0)).shrink(0.5);
            let fill = if p[2].abs() > 0.1 { egui::Color32::from_rgb(205, 185, 140) } else { egui::Color32::from_rgb(225, 205, 160) };
            painter.rect_filled(r, 3.0, fill);
            painter.rect_stroke(r, 3.0, egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(90, 75, 50)), egui::StrokeKind::Inside);
            let label = labels.get(i).cloned().unwrap_or_else(|| format!("{}", i + 1));
            painter.text(r.center(), egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(10.0), egui::Color32::BLACK);
        }
    });
}

/// Label of each slot of the first two layers: the pick number, with a/b for the
/// two drops of a pick that places its products one at a time.
fn pick_labels(p: &Pattern) -> Vec<String> {
    let first_two = (p.odd.len() + if p.layers > 1 { p.even.len() } else { 0 }) as u32;
    let mut labels = vec![String::new(); first_two as usize];
    for (k, pick) in p.picks.iter().enumerate() {
        for (d, drop) in pick.iter().enumerate() {
            for &slot in drop {
                if slot < first_two {
                    labels[slot as usize] = if pick.len() > 1 {
                        format!("{}{}", k + 1, if d == 0 { 'a' } else { 'b' })
                    } else {
                        format!("{}", k + 1)
                    };
                }
            }
        }
    }
    labels
}

/// The window. `tdps` are the palletizing TDP names; returns the pattern to apply
/// to `target` when the user clicks "Aplicar".
/// `sized`: emitters and mesas (name, kind label, current box size in m).
pub fn pattern_window(
    ctx: &egui::Context,
    state: &mut PatternUi,
    tdps: &[(String, f32)],
    mesa_box: Option<[f32; 3]>,
    sized: &[(String, &str, [f32; 3])],
) -> Option<(String, Pattern)> {
    let mut apply = None;
    let mut open = state.open;
    egui::Window::new("Padrões de paletização")
        .open(&mut open)
        .default_width(760.0)
        .default_height(620.0)
        .vscroll(true)
        .show(ctx, |ui| {
            if state.auto_apply
                && let Some(bx) = mesa_box
            {
                // Automated test: the mesa fardo, the cabeçote with two, 3 layers.
                state.input.product = [bx[0] * 1000.0, bx[2] * 1000.0, bx[1] * 1000.0];
                state.input.per_pick = 2;
                state.input.layers = 3;
                state.input.recipe_10x1 = std::env::var("PDT_PATTERN").is_ok_and(|v| v == "apply10x1");
            }
            let input = &mut state.input;
            ui.columns(2, |cols| {
                egui::Grid::new("pattern_product").num_columns(2).show(&mut cols[0], |ui| {
                    ui.strong("Produto");
                    if let Some(bx) = mesa_box {
                        if ui.small_button("usar fardo da mesa").clicked() {
                            input.product = [bx[0] * 1000.0, bx[2] * 1000.0, bx[1] * 1000.0];
                            input.per_pick = 2;
                            input.pick_in_line = false;
                        }
                    } else {
                        ui.label("");
                    }
                    ui.end_row();
                    mm_field(ui, "Comprimento", &mut input.product[0], 3000.0, " mm");
                    mm_field(ui, "Largura", &mut input.product[1], 3000.0, " mm");
                    mm_field(ui, "Altura", &mut input.product[2], 3000.0, " mm");
                    ui.label("Peso");
                    ui.add(egui::DragValue::new(&mut input.product_kg).speed(0.1).range(0.0..=2000.0).suffix(" kg"));
                    ui.end_row();
                    ui.strong("Cabeçote do robô");
                    ui.label("");
                    ui.end_row();
                    ui.label("Produtos na garra");
                    ui.add(egui::DragValue::new(&mut input.per_pick).range(1..=2))
                        .on_hover_text("Cabeçote com duas pinças: cada lado segura um produto e abre sozinho");
                    ui.end_row();
                    ui.label("Arranjo da pega");
                    ui.horizontal(|ui| {
                        ui.radio_value(&mut input.pick_in_line, false, "lado a lado");
                        ui.radio_value(&mut input.pick_in_line, true, "em linha");
                    });
                    ui.end_row();
                    mm_field(ui, "Folga na pega", &mut input.pick_gap, 200.0, " mm");
                    ui.label("Mesa 3 estágios");
                    ui.checkbox(&mut input.recipe_10x1, "receita 10x1 (pegas de 4 e 3)").on_hover_text(
                        "A mesa forma 4 fardos lado a lado (o robô deposita 2 numa ponta e 2 na outra), depois 3 e 3",
                    );
                    ui.end_row();
                    ui.label("Por pega");
                    ui.strong(if input.recipe_10x1 {
                        "4 (2 + 2 nas pontas), 3, 3"
                    } else if input.per_pick >= 2 {
                        "2 produtos (par ou dois depósitos simples)"
                    } else {
                        "1 produto"
                    });
                    ui.end_row();
                });
                egui::Grid::new("pattern_pallet").num_columns(2).show(&mut cols[1], |ui| {
                    ui.strong("Pallet");
                    let current = PALLETS.iter().find(|(_, s)| *s == input.pallet).map_or("Personalizado", |(n, _)| *n);
                    egui::ComboBox::from_id_salt("pallet_preset").selected_text(current).width(200.0).show_ui(ui, |ui| {
                        for (name, size) in PALLETS {
                            if ui.selectable_label(input.pallet == *size, *name).clicked() {
                                input.pallet = *size;
                            }
                        }
                    });
                    ui.end_row();
                    mm_field(ui, "Comprimento (sentido do TDP)", &mut input.pallet[0], 3000.0, " mm");
                    mm_field(ui, "Largura", &mut input.pallet[1], 3000.0, " mm");
                    mm_field(ui, "Altura do pallet", &mut input.pallet[2], 500.0, " mm");
                    mm_field(ui, "Sobra permitida por lado", &mut input.overhang, 200.0, " mm");
                    mm_field(ui, "Folga entre unidades", &mut input.gap, 100.0, " mm");
                    ui.strong("Limites");
                    ui.label("");
                    ui.end_row();
                    mm_field(ui, "Altura máxima (com pallet)", &mut input.max_height, 4000.0, " mm");
                    ui.label("Peso máximo");
                    ui.add(egui::DragValue::new(&mut input.max_kg).speed(10.0).range(0.0..=5000.0).suffix(" kg"));
                    ui.end_row();
                    ui.label("Camadas");
                    ui.add(egui::DragValue::new(&mut input.layers).range(0..=40))
                        .on_hover_text("0 = o máximo permitido pela altura e pelo peso");
                    ui.end_row();
                });
            });

            if state.generated_for.as_ref() != Some(&state.input) {
                state.candidates = generate(&state.input);
                state.generated_for = Some(state.input.clone());
                state.selected = 0;
            }
            ui.separator();
            if state.candidates.is_empty() {
                ui.colored_label(egui::Color32::from_rgb(230, 120, 60), "Nenhum formato cabe no pallet com essas medidas.");
                return;
            }
            let pallet = (state.input.pallet[0] / 1000.0, state.input.pallet[1] / 1000.0);
            for (i, c) in state.candidates.iter().enumerate() {
                let selected = i == state.selected;
                let frame = egui::Frame::group(ui.style()).stroke(if selected {
                    egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(60, 160, 230))
                } else {
                    ui.visuals().widgets.noninteractive.bg_stroke
                });
                let response = frame
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            let labels = pick_labels(&c.pattern);
                            let n = c.pattern.odd.len();
                            draw_layer(ui, "1ª camada (TDP para a direita)", &c.pattern.odd, &labels[..n.min(labels.len())], c.pattern.unit, pallet, 150.0);
                            draw_layer(ui, "2ª camada", &c.pattern.even, labels.get(n..).unwrap_or(&[]), c.pattern.unit, pallet, 150.0);
                            ui.vertical(|ui| {
                                ui.heading(&c.pattern.name);
                                ui.label(format!(
                                    "{} por camada × {} camadas = {} unidades",
                                    c.per_layer, c.pattern.layers, c.total
                                ));
                                ui.label(format!(
                                    "Altura {:.0} mm · peso {:.0} kg (limitado por {})",
                                    c.height, c.kg, c.limit
                                ));
                                ui.label(format!("Ocupação da área do pallet {:.0}%", c.area_use));
                                let picks = &c.pattern.picks;
                                let pairs = picks.iter().filter(|k| k.len() == 1 && k[0].len() == 2).count();
                                let singles = picks.iter().filter(|k| k.len() == 2).count();
                                let mut text = if c.pattern.groups.is_empty() {
                                    format!("{} pegas: {} em par, {} com dois depósitos", picks.len(), pairs, singles)
                                } else {
                                    let sizes: Vec<String> = picks.iter().take(6).map(|k| k.iter().map(|d| d.len().to_string()).collect::<Vec<_>>().join("+")).collect();
                                    format!("{} pegas ({}…)", picks.len(), sizes.join(", "))
                                };
                                if picks.last().is_some_and(|k| k.iter().flatten().count() == 1) {
                                    text.push_str(", última com 1 produto");
                                }
                                ui.label(text).on_hover_text("Número = pega do robô; a/b = depósitos da mesma pega (abre um lado da garra por vez)");
                                if !c.fits_groups {
                                    ui.colored_label(
                                        egui::Color32::from_rgb(230, 120, 60),
                                        "Não fecha as pegas da mesa (4 = 2 + 2, 3, 3): sobram fardos colocados um a um",
                                    );
                                }
                                if c.overhang > 0.5 {
                                    ui.label(format!("Sobra máxima {:.0} mm por lado", c.overhang));
                                }
                                ui.label(if c.interlocked {
                                    "Camadas amarradas: juntas desencontradas, mais estável"
                                } else {
                                    "Camadas iguais: juntas alinhadas, precisa de filme/papelão"
                                });
                                if ui.radio(selected, "Escolher este formato").clicked() {
                                    state.selected = i;
                                }
                            });
                        });
                    })
                    .response;
                if response.interact(egui::Sense::click()).clicked() {
                    state.selected = i;
                }
            }
            if state.auto_apply && !tdps.is_empty() {
                state.auto_apply = false;
                state.target = tdps[0].0.clone();
                apply = Some((state.target.clone(), state.candidates[0].pattern.clone()));
                state.close_after = true;
            }
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Aplicar em");
                egui::ComboBox::from_id_salt("pattern_target")
                    .selected_text(if state.target.is_empty() { "(escolha o TDP)" } else { &state.target })
                    .show_ui(ui, |ui| {
                        for (name, _) in tdps {
                            ui.selectable_value(&mut state.target, name.clone(), name);
                        }
                    });
                let can = !state.target.is_empty() && state.selected < state.candidates.len();
                if ui.add_enabled(can, egui::Button::new("Aplicar ao TDP de paletização")).clicked() {
                    let pattern = state.candidates[state.selected].pattern.clone();
                    state.message = format!("\"{}\" aplicado em {}.", pattern.name, state.target);
                    apply = Some((state.target.clone(), pattern));
                }
            });
            // Top of the finished pallet on the chosen TDP, against the robot reach.
            if let (Some((_, tdp_h)), Some(c)) = (tdps.iter().find(|(n, _)| *n == state.target), state.candidates.get(state.selected)) {
                let top = tdp_h + c.height / 1000.0;
                // The TCP is at the bottom of the fardos (clamp tips): the top of the
                // last layer is one product higher.
                let reach = crate::robot::spec::MAX_HEIGHT - crate::robot::TOOL_LENGTH - crate::robot::JAW_HEIGHT + c.pattern.unit[1];
                let text = format!("No {}, o topo do pallet completo fica a {:.2} m do piso.", state.target, top);
                if top > reach {
                    ui.colored_label(
                        egui::Color32::from_rgb(230, 120, 60),
                        format!("{text} Acima do alcance do robô com o cabeçote (topo da camada até {reach:.2} m): reduza as camadas ou a altura máxima."),
                    );
                } else {
                    ui.label(text);
                }
            }
            ui.weak("Cada posição é um produto. O número é a pega do robô; a/b são os depósitos de uma pega que abre uma pinça de cada vez.");
            if !state.message.is_empty() {
                ui.label(&state.message);
            }

            // The product size on the emitters (boxes created) and mesas (lanes, empurrador).
            ui.separator();
            ui.strong("Produto nos emissores e mesas");
            let [l, w, h] = state.input.product;
            let product_box = [l / 1000.0, h / 1000.0, w / 1000.0];
            ui.weak(format!("Tamanho do produto: {l:.0} × {w:.0} × {h:.0} mm (comprimento × largura × altura)"));
            if sized.is_empty() {
                ui.weak("Nenhum emissor de caixas ou mesa na célula.");
            }
            for (name, kind, current) in sized {
                let same = (0..3).all(|i| (current[i] - product_box[i]).abs() < 0.0005);
                let text = format!(
                    "{name} ({kind}): atual {:.0} × {:.0} × {:.0} mm{}",
                    current[0] * 1000.0,
                    current[2] * 1000.0,
                    current[1] * 1000.0,
                    if same { " (igual)" } else { "" }
                );
                let mut on = state.size_targets.contains(name);
                if ui.checkbox(&mut on, text).changed() {
                    if on {
                        state.size_targets.push(name.clone());
                    } else {
                        state.size_targets.retain(|n| n != name);
                    }
                }
            }
            ui.horizontal(|ui| {
                if ui.button("Marcar todos").clicked() {
                    state.size_targets = sized.iter().map(|(n, ..)| n.clone()).collect();
                }
                let any = sized.iter().any(|(n, ..)| state.size_targets.contains(n));
                if ui.add_enabled(any, egui::Button::new("Atualizar tamanho do produto")).clicked() {
                    let names: Vec<String> = sized.iter().map(|(n, ..)| n.clone()).filter(|n| state.size_targets.contains(n)).collect();
                    state.message = format!("Produto {l:.0} × {w:.0} × {h:.0} mm em {}.", names.join(", "));
                    state.size_request = Some((names, product_box));
                }
            });
        });
    state.open = open && !std::mem::take(&mut state.close_after);
    apply
}

#[cfg(test)]
mod tests {
    use super::*;

    fn overlaps(places: &[[f32; 3]], unit: (f32, f32)) -> bool {
        places.iter().enumerate().any(|(i, p)| {
            places[i + 1..].iter().any(|q| {
                let (pa, pb) = footprint(p, unit);
                let (qa, qb) = footprint(q, unit);
                (p[0] - q[0]).abs() < (pa + qa) / 2.0 - 1e-3 && (p[1] - q[1]).abs() < (pb + qb) / 2.0 - 1e-3
            })
        })
    }

    #[test]
    fn options_fit_without_overlap() {
        for (product, pallet) in [
            ([400.0, 300.0, 250.0], [1000.0, 1200.0, 144.0]),
            ([600.0, 400.0, 150.0], [1000.0, 1200.0, 144.0]),
            ([500.0, 350.0, 120.0], [1100.0, 1100.0, 144.0]),
        ] {
            let input = PatternInput { product, pallet, per_pick: 1, ..default() };
            let options = generate(&input);
            assert!(!options.is_empty());
            for c in &options {
                let u = (c.pattern.unit[0], c.pattern.unit[2]);
                assert!(!overlaps(&c.pattern.odd, u), "{} odd", c.pattern.name);
                assert!(!overlaps(&c.pattern.even, u), "{} even", c.pattern.name);
                assert!(c.overhang < 0.5, "{} sobra {}", c.pattern.name, c.overhang);
            }
        }
    }

    #[test]
    fn bag_pattern_beats_column() {
        // 600 x 400 bags on a 1200 x 1000 pallet: 5 per layer interlocked (column fits 4).
        let input = PatternInput { product: [600.0, 400.0, 150.0], pallet: [1200.0, 1000.0, 144.0], per_pick: 1, ..default() };
        let options = generate(&input);
        let column = options.iter().find(|c| c.pattern.name == "Coluna").unwrap();
        let best = &options[0];
        assert_eq!(column.per_layer, 4);
        assert_eq!(best.per_layer, 5);
        assert!(best.interlocked);
    }

    #[test]
    fn odd_layer_picks_like_the_recipe() {
        // 5 products per layer (2 crosswise that cannot pair, 3 lengthwise in a column):
        // pick 1 = two simple drops, pick 2 = a pair, pick 3 = the last one + the
        // first of the next layer; the last product of the pallet is a single pick.
        let input = PatternInput {
            product: [520.0, 350.0, 110.0],
            pallet: [1000.0, 1200.0, 144.0],
            per_pick: 2,
            pick_gap: 0.0,
            layers: 3,
            ..default()
        };
        let c = generate(&input).into_iter().find(|c| c.per_layer == 5 && c.interlocked).expect("5 por camada");
        let picks = &c.pattern.picks;
        assert_eq!(picks.iter().flatten().flatten().count(), 15);
        assert_eq!(picks[0].len(), 2, "{picks:?}");
        assert_eq!(picks[1], vec![picks[1][0].clone()]);
        assert_eq!(picks[1][0].len(), 2, "{picks:?}");
        assert_eq!(picks[2].len(), 2, "{picks:?}");
        assert!(picks[2][0][0] < 5 && picks[2][1][0] >= 5, "{picks:?}");
        assert_eq!(picks.last().unwrap().iter().flatten().count(), 1, "{picks:?}");
        // Every slot exactly once.
        let mut all: Vec<u32> = c.pattern.sequence();
        all.sort();
        assert_eq!(all, (0..15).collect::<Vec<_>>());
    }

    #[test]
    fn recipe_10x1_groups() {
        // 10 per layer (two rows of 5 side by side): 4 = 2 + 2 at the ends, then 3 and 3.
        let input = PatternInput {
            product: [500.0, 200.0, 110.0],
            pallet: [1000.0, 1200.0, 144.0],
            recipe_10x1: true,
            layers: 2,
            ..default()
        };
        let c = generate(&input).into_iter().next().expect("formato");
        assert!(c.fits_groups && c.per_layer == 10, "{} {}", c.pattern.name, c.per_layer);
        let picks = &c.pattern.picks;
        assert_eq!(picks.len(), 6, "{picks:?}");
        for (i, pick) in picks.iter().enumerate() {
            let sizes: Vec<usize> = pick.iter().map(|d| d.len()).collect();
            let expected: &[usize] = if i % 3 == 0 { &[2, 2] } else { &[3] };
            assert_eq!(sizes, expected, "{picks:?}");
        }
        let mut all = c.pattern.sequence();
        all.sort();
        assert_eq!(all, (0..20).collect::<Vec<_>>());
    }

    #[test]
    fn slots_walk_the_layers() {
        let input = PatternInput { layers: 2, ..default() };
        let c = &generate(&input)[0];
        let n = c.per_layer as u32;
        let (first, _) = c.pattern.slot(0.144, 0.25, 0).unwrap();
        let (second_layer, _) = c.pattern.slot(0.144, 0.25, n).unwrap();
        assert!((second_layer.y - first.y - 0.25).abs() < 1e-5);
        assert!(c.pattern.slot(0.144, 0.25, 2 * n).is_none());
    }
}
