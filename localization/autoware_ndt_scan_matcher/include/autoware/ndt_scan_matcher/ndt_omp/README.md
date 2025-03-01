# ndt_omp

<!-->

cspell:ignore Kenji, Koide
<-->

The codes in this directory are derived from <https://github.com/tier4/ndt_omp>, which is a fork of <https://github.com/koide3/ndt_omp>.

We use this code in accordance with the LICENSE, and we have directly confirmed this with Dr. Kenji Koide.

We sincerely appreciate Dr. Koide’s support and contributions.

## nanoflann

We have copied [nanoflann.hpp](https://github.com/jlblancoc/nanoflann) to use the faster KDTree.

`nanoflann.hpp` in this directory is completely identical to the original `nanoflann.hpp`.

The copy procedure is:

```bash
# pwd = ~/work
git clone https://github.com/jlblancoc/nanoflann
cd nanoflann
# download [git-filter-repo](https://github.com/newren/git-filter-repo/blob/main/git-filter-repo)
python3 ~/Downloads/git-filter-repo.py --path include/nanoflann.hpp
cd ~/autoware/src/universe/autoware.universe/
git remote add nanoflann ~/work/nanoflann
git fetch nanoflann
git merge --allow-unrelated-histories nanoflann/master
mv include/nanoflann ./localization/autoware_ndt_scan_matcher/include/autoware/ndt_scan_matcher/ndt_omp/
```
