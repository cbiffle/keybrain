use std::rc::Rc;

static COLS: [u8; 19] = [
    0b1100_0000, // 0
    0b1000_0001,
    0b1000_0101,
    0b0001_1000,
    0b1111_1111,
    0b1101_1111, // 5
    0b1111_1101,
    0b1111_1010,
    0b1111_1110,
    0b1100_0101,
    0b1111_1111, // 10
    0b1110_0010,
    0b0111_0101,
    0b1100_1111,
    0b0000_1000,
    0b0001_0000, // 15
    0b1111_1110,
    0b1111_1110,
    0b1111_1110,
];

fn main() {
    println!("commencing search");
    let mut best = (COLS.len(), COLS.to_vec(), None);
    search(&COLS, COLS.len(), None, &mut best);

    println!("best score: {}", best.0);
    println!("minimum column loading: {}", best.1.iter().map(|x| x.count_ones()).fold(8, |x, y| x.min(y)));

    let mut trail = best.2;
    while let Some(step) = &trail {
        println!("- connect {} and {} ({:08b})", step.i, step.j, step.result);
        trail = step.trail.clone();
    }
}

#[derive(Debug)]
struct Move {
    i: usize,
    j: usize,
    result: u8,
    trail: Option<Rc<Move>>,
}

fn search(state: &[u8], score: usize, trail: Option<Rc<Move>>, best: &mut (usize, Vec<u8>, Option<Rc<Move>>)) {
    if score <= best.0 {
        println!("{}\t{:?}", score, trail);
        *best = (score, state.to_vec(), trail.clone());
    }
    for (i, &ci) in state.iter().enumerate() {
        for (ij, &cj) in state[i+1..].iter().enumerate() {
            let j = ij + i + 1;
            if nonoverlapping(ci, cj) {
                let mut new_state = state.to_vec();
                new_state[i] |= cj;
                new_state[j] = 0xFF;
                let new_trail = Rc::new(Move {
                    i,
                    j,
                    result: new_state[i],
                    trail: trail.clone(),
                });
                search(&new_state, score - 1, Some(new_trail), best);
            }
        }
    }
}

fn nonoverlapping(a: u8, b: u8) -> bool {
    (a & b) == 0
}
