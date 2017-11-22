uint8_t debouce(uint8_t current, InputData &input) {
  if (input.state != current) {
    input.count = input.count + 1;
    if (input.count > debounceCount) {
      input.count = 0;
      input.state = current;
    }
  } else {
    input.count = 0;
  }
  return input.state;
}
