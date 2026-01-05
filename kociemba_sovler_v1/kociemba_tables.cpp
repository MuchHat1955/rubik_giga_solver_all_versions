#include "kociemba_tables.h"

namespace kociemba {

short *twistMove = nullptr;
short *flipMove = nullptr;

short *FRtoBR_Move = nullptr;
short *FRtoBR_Move24 = nullptr;
short *URFtoDLF_Move = nullptr;
short *URtoDF_Move = nullptr;
short *UBtoDF_Move = nullptr;

signed char *Slice_URFtoDLF_Parity_Prun = nullptr;
signed char *Slice_URtoDF_Parity_Prun = nullptr;
signed char *Slice_Flip_Prun = nullptr;
signed char *Slice_Twist_Prun = nullptr;

unsigned char *Slice_Flip_Prun_fast = nullptr;
unsigned char *Slice_Twist_Prun_fast = nullptr;

}

static bool load_bin(const char *path, void *dst, size_t bytes) {
  //TODO enable the code below once the SD is installed

  /*
  File f = SD.open(path, FILE_READ);
  if (!f) return false;

  size_t r = f.read((uint8_t*)dst, bytes);
  f.close();

  return r == bytes;*/
  return false;
}

bool kociemba_init_tables() {
  using namespace kociemba;

  twistMove = (short *)malloc(twistMove_BYTES);
  flipMove = (short *)malloc(flipMove_BYTES);
  FRtoBR_Move = (short *)malloc(FRtoBR_Move_BYTES);
  FRtoBR_Move24 = (short *)malloc(FRtoBR_Move24_BYTES);
  URFtoDLF_Move = (short *)malloc(URFtoDLF_Move_BYTES);
  URtoDF_Move = (short *)malloc(URtoDF_Move_BYTES);
  UBtoDF_Move = (short *)malloc(UBtoDF_Move_BYTES);

  Slice_URFtoDLF_Parity_Prun = (signed char *)malloc(Slice_URFtoDLF_Parity_Prun_BYTES);
  Slice_URtoDF_Parity_Prun = (signed char *)malloc(Slice_URtoDF_Parity_Prun_BYTES);
  Slice_Flip_Prun = (signed char *)malloc(Slice_Flip_Prun_BYTES);
  Slice_Twist_Prun = (signed char *)malloc(Slice_Twist_Prun_BYTES);

  Slice_Flip_Prun_fast = (unsigned char *)malloc(Slice_Flip_Prun_fast_BYTES);
  Slice_Twist_Prun_fast = (unsigned char *)malloc(Slice_Twist_Prun_fast_BYTES);

  if (!twistMove || !flipMove || !FRtoBR_Move || !URFtoDLF_Move || !URtoDF_Move || !Slice_Flip_Prun || !Slice_Twist_Prun)
    return false;

  return load_bin("/kociemba/twistMove.bin", twistMove, twistMove_BYTES) && load_bin("/kociemba/flipMove.bin", flipMove, flipMove_BYTES) && load_bin("/kociemba/FRtoBR_Move.bin", FRtoBR_Move, FRtoBR_Move_BYTES) && load_bin("/kociemba/FRtoBR_Move24.bin", FRtoBR_Move24, FRtoBR_Move24_BYTES) && load_bin("/kociemba/URFtoDLF_Move.bin", URFtoDLF_Move, URFtoDLF_Move_BYTES) && load_bin("/kociemba/URtoDF_Move.bin", URtoDF_Move, URtoDF_Move_BYTES) && load_bin("/kociemba/UBtoDF_Move.bin", UBtoDF_Move, UBtoDF_Move_BYTES) && load_bin("/kociemba/Slice_URFtoDLF_Parity_Prun.bin", Slice_URFtoDLF_Parity_Prun, Slice_URFtoDLF_Parity_Prun_BYTES) && load_bin("/kociemba/Slice_URtoDF_Parity_Prun.bin", Slice_URtoDF_Parity_Prun, Slice_URtoDF_Parity_Prun_BYTES) && load_bin("/kociemba/Slice_Flip_Prun.bin", Slice_Flip_Prun, Slice_Flip_Prun_BYTES) && load_bin("/kociemba/Slice_Twist_Prun.bin", Slice_Twist_Prun, Slice_Twist_Prun_BYTES);
}
