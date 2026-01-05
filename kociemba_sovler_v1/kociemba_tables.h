namespace kociemba {

constexpr size_t twistMove_BYTES =
    N_TWIST * N_MOVE * sizeof(short);

constexpr size_t flipMove_BYTES =
    N_FLIP * N_MOVE * sizeof(short);

constexpr size_t FRtoBR_Move_BYTES =
    N_FRtoBR * N_MOVE * sizeof(short);

constexpr size_t FRtoBR_Move24_BYTES =
    (N_FRtoBR / 24) * N_MOVE * sizeof(short);

constexpr size_t URFtoDLF_Move_BYTES =
    N_URFtoDLF * N_MOVE * sizeof(short);

constexpr size_t URtoDF_Move_BYTES =
    N_URtoDF * N_MOVE * sizeof(short);

constexpr size_t UBtoDF_Move_BYTES =
    N_UBtoDF * N_MOVE * sizeof(short);

constexpr size_t Slice_URFtoDLF_Parity_Prun_BYTES =
    N_SLICE2 * N_URFtoDLF * N_PARITY / 2;

constexpr size_t Slice_URtoDF_Parity_Prun_BYTES =
    N_SLICE2 * N_URtoDF * N_PARITY / 2;

constexpr size_t Slice_Flip_Prun_BYTES =
    N_SLICE1 * N_FLIP / 2;

constexpr size_t Slice_Twist_Prun_BYTES =
    N_SLICE1 * N_TWIST / 2 + 1;

constexpr size_t Slice_Flip_Prun_fast_BYTES =
    N_SLICE1 * N_FLIP / 4;

constexpr size_t Slice_Twist_Prun_fast_BYTES =
    N_SLICE1 * N_TWIST / 4 + 1;

}
