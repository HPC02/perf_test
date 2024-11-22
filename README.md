# README #

## 1. test_array_update -- Memory Bound Test ##

This test load large number of samples from data file into a `vector`(named `ds_append_`).
Then create another `vector` with the same size(named `ds_`), update each element of `ds_` from `ds_append_` with some arithmetic operation.

Test data file `src/test_array_update/test_data/test.dataxy` should be placed in the same directory as the executable `test_array_update`.
