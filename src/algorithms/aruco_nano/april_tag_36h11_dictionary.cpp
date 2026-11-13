#include "ppmt_fiducial_core/algorithms/aruco_nano/april_tag_36h11_dictionary.hpp"

#include <algorithm>
#include <tuple>

namespace ppmt::fiducial
{

AprilTag36h11Dictionary::AprilTag36h11Dictionary()
{
  std::size_t entry_index = 0;
  for (int id = 0; id < april_tag_36h11_code_count; ++id) {
    for (int rotation = 0; rotation < rotation_count; ++rotation) {
      entries_sorted_by_code_[entry_index++] = {april_tag_36h11_codes[id][rotation], id, rotation};
    }
  }
  std::sort(entries_sorted_by_code_.begin(), entries_sorted_by_code_.end(),
            [](const CodeEntry & left, const CodeEntry & right) {
              return std::tie(left.code, left.id, left.rotation) <
                     std::tie(right.code, right.id, right.rotation);
            });
}

bool AprilTag36h11Dictionary::identify(std::uint64_t code, int & id, int & rotation) const
{
  // First entry with this code = lowest id, then lowest rotation.
  const auto found = std::lower_bound(
    entries_sorted_by_code_.begin(), entries_sorted_by_code_.end(), code,
    [](const CodeEntry & entry, std::uint64_t searched_code) { return entry.code < searched_code; });
  if (found == entries_sorted_by_code_.end() || found->code != code) {
    return false;
  }
  id = found->id;
  rotation = found->rotation;
  return true;
}

}  // namespace ppmt::fiducial
