//! Debouncing algorithm.
//!
//! We model the bounce of each key switch separately, because that's how
//! physics works.

use core::sync::atomic::{AtomicU8, Ordering};

const DELAY: u8 = 6;

/// Describes the state of a single key being debounced.
#[derive(Copy, Clone, Debug)]
pub enum KeyState {
    /// The debouncing interval has elapsed since the last change, and the key
    /// is considered to be stably in the given state.
    Stable(LogicalState),
    /// The key has changed recently and is transitioning into the given state,
    /// after the given number of periods.
    Unstable(LogicalState, u8),
}

impl KeyState {
    /// Advances the debouncing state machine, given the current instantaneous
    /// state of the switch, `current_state`.
    pub fn step(&mut self, current_state: LogicalState) {
        match self {
            // When in an apparently stable state, transition away as soon as
            // current_state is observed to change.
            Self::Stable(state) => {
                if *state != current_state {
                    *self = Self::Unstable(state.complement(), DELAY);
                }
            }

            // This case terminates the debouncing interval. Note that, in what
            // is almost certainly overengineering, we will short-circuit back
            // to the opposing Unstable state if current_state is still
            // changing.
            Self::Unstable(tgt, 0) => {
                if *tgt == current_state {
                    *self = KeyState::Stable(*tgt);
                } else {
                    *self = KeyState::Unstable(tgt.complement(), DELAY);
                }
            }

            // This line is the key to the debouncing method: until the counter
            // in the Unstable state has ticked down to zero, we *do not* look
            // at current_state.
            Self::Unstable(tgt, n) => *self = Self::Unstable(*tgt, *n - 1),

        }
    }

    /// Checks whether this key should be treated as open or closed, for keycode
    /// generation purposes.
    pub fn state(&self) -> LogicalState {
        // A key is considered closed from the beginning of its debouncing
        // interval, after a closing edge has been observed, and for the
        // duration of its stable closed state. As soon as an opening edge is
        // observed, the key is treated as open.
        match self {
            Self::Unstable(s, _) | Self::Stable(s) => *s,
        }
    }

    /// Shorthand for asking if the key is down.
    pub fn is_closed(&self) -> bool {
        self.state() == LogicalState::Closed
    }
}

impl Default for KeyState {
    fn default() -> Self {
        Self::Stable(LogicalState::Open)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum LogicalState { Open, Closed }

impl LogicalState {
    pub fn complement(self) -> Self {
        match self {
            Self::Open => Self::Closed,
            Self::Closed => Self::Open,
        }
    }
}
