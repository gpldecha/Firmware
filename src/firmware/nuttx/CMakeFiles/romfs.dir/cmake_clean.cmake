file(REMOVE_RECURSE
  "romfs.o"
  "extras/px4io-v2.bin"
  "libromfs.pdb"
  "libromfs.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/romfs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
