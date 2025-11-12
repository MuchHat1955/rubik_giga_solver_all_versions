#pragma once
#include <Arduino.h>
#include <vector>
#include <string>
#include "pose_store.h"

class SequenceStore {
public:
  struct Sequence {
    const char* key;                 // sequence key (e.g., "bottom_plus")
    const char* text;                // label or display text
    std::vector<const char*> poses;  // ordered list of pose names
  };

  SequenceStore();

  bool run_sequence_by_key(const char* key, const char* name);
  bool is_key_for_sequence(const char* key) const;
  String listAllSequences() const;

private:
  std::vector<Sequence> sequences;
};

extern SequenceStore sequence_store;
