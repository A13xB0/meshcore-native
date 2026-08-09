// The littlefs internals the companion application reaches for.
//
// It reports storage use to its client, and it does so by traversing the
// filesystem's block allocation rather than by asking for a byte count. That is
// the right thing to do on a flash part and there is no way to answer it from
// the POSIX API alone — so the variant presents the same block geometry the
// nRF52840 build has, and counts blocks the way littlefs would.
//
// Same geometry as the emulated target on purpose: if the two backends reported
// different capacities, a scenario that filled the store would behave
// differently on each, and the cross-check between them would be comparing two
// different devices.
#pragma once

#include <dirent.h>
#include <stdint.h>
#include <sys/stat.h>

#include <string>

using lfs_block_t = uint32_t;
using lfs_size_t = uint32_t;
using lfs_ssize_t = int32_t;

#define LFS_ERR_CORRUPT (-84)

struct lfs_config {
  lfs_size_t block_size;
  lfs_block_t block_count;
};

struct lfs {
  const lfs_config* cfg;
  std::string root;
};

// Adafruit's InternalFS on an nRF52840: 28 blocks of 4 KiB, the last pages of
// the 1 MiB flash.
inline const lfs_config* lfs_host_config() {
  static const lfs_config cfg{4096, 28};
  return &cfg;
}

// Walks every allocated block, calling back once per block, as lfs_traverse
// does. A file of n bytes occupies ceil(n / block_size) blocks, plus one for its
// metadata — which is what makes a store of many small files fill up long before
// its byte total suggests, and is exactly the behaviour worth reproducing.
inline int lfs_traverse(lfs* fs, int (*cb)(void*, lfs_block_t), void* data) {
  if (!fs) return 0;
  DIR* d = opendir(fs->root.c_str());
  if (!d) return 0;
  lfs_block_t block = 0;
  int err = 0;
  while (struct dirent* e = readdir(d)) {
    if (e->d_name[0] == '.') continue;
    struct stat st{};
    if (stat((fs->root + "/" + e->d_name).c_str(), &st) != 0) continue;
    lfs_size_t used = ((lfs_size_t)st.st_size + fs->cfg->block_size - 1) / fs->cfg->block_size + 1;
    for (lfs_size_t i = 0; i < used; i++) {
      if ((err = cb(data, block++)) != 0) {
        closedir(d);
        return err;
      }
    }
  }
  closedir(d);
  return 0;
}
