^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_probabilistic_occupancy_grid_map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_probabilistic_occupancy_grid_map): cuda accelerated implementation (`#9542 <https://github.com/autowarefoundation/autoware.universe/issues/9542>`_)
  * feat: implemented a cuda accelerated ogm
  * chore: fixed cspells
  * chore: unused header and variable names
  * chore: cspell fixes
  * chore: cppcheck fix attempt
  * fix: attempting to fix ci/cd regarding the cuda library
  * chore: fixed the order of the cuda check in the cmakelist
  * fix: removed cuda as a required dep for cpu only builds
  * fix: missing cuda linking (?)
  * feat: fixed single mode, added streams, and added the restrict keyword
  * chore: replaced a potential indetermination using an epsilon
  * Update perception/autoware_probabilistic_occupancy_grid_map/lib/updater/log_odds_bayes_filter_updater_kernel.cu
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
  * Update perception/autoware_probabilistic_occupancy_grid_map/lib/updater/log_odds_bayes_filter_updater_kernel.cu
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
  * style(pre-commit): autofix
  * chore: added bound checkings in the update origin kernel
  * chore: disabled tests since universe does not support cuda in ci/cd
  * chore: added me as a maintainer
  * fix: missedn the end in the cmake
  * chore: moved the boudnary checks to only the cuda version since the cpu version uses the upstream logic
  ---------
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_probabilistic_occupancy_grid_map): tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_probabilistic_occupancy_grid_map (`#9895 <https://github.com/autowarefoundation/autoware.universe/issues/9895>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files perception/autoware_probabilistic_occupancy_grid_map
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix(perception): fix perception docs (`#9766 <https://github.com/autowarefoundation/autoware.universe/issues/9766>`_)
  * fix: fix perception docs
  * fix: fix missing parameter in schema
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_object_merger/schema/data_association_matrix.schema.json
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * style(pre-commit): autofix
  * chore: seperate paramters for different nodes
  ---------
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_probabilistic_occupancy_grid_map): fix bugprone-branch-clone (`#9652 <https://github.com/autowarefoundation/autoware.universe/issues/9652>`_)
  fix: bugprone-error
* Contributors: Fumiya Watanabe, Kenzo Lobos Tsunekawa, Vishal Chauhan, Yi-Hsiang Fang (Vivid), kobayu858

0.40.0 (2024-12-12)
-------------------
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware.universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware.universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware.universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware.universe/issues/9588>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_probabilistic_occupancy_grid_map): fix bugprone-incorrect-roundings (`#9221 <https://github.com/autowarefoundation/autoware.universe/issues/9221>`_)
  fix: bugprone-incorrect-roundings
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_probabilistic_occupancy_grid_map): fix bugprone-incorrect-roundings (`#9221 <https://github.com/autowarefoundation/autoware.universe/issues/9221>`_)
  fix: bugprone-incorrect-roundings
* Contributors: Esteve Fernandez, Yutaka Kondo, kobayu858

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(probabilistic_occupancy_grid_map): add time_keeper (`#8601 <https://github.com/autowarefoundation/autoware.universe/issues/8601>`_)
  * add time_keeper
  * add option for time keeper
  * correct namespace
  * set default to false
  * add scope and timekeeper
  * remove scope and add comment for scopes
  * mod comment
  * change comment
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * fix variable shadowing
  ---------
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* fix(autoware_probabilistic_occupancy_grid_map): fix unusedFunction (`#8574 <https://github.com/autowarefoundation/autoware.universe/issues/8574>`_)
  fix:unusedFunction
* fix(autoware_probabilistic_occupancy_grid_map): fix functionConst (`#8426 <https://github.com/autowarefoundation/autoware.universe/issues/8426>`_)
  fix:functionConst
* fix(autoware_probabilistic_occupancy_grid_map): fix uninitMemberVar (`#8333 <https://github.com/autowarefoundation/autoware.universe/issues/8333>`_)
  fix:uninitMemberVar
* fix(autoware_probabilistic_occupancy_grid_map): fix functionConst (`#8289 <https://github.com/autowarefoundation/autoware.universe/issues/8289>`_)
  fix:functionConst
* refactor(probabilistic_occupancy_grid_map, occupancy_grid_map_outlier_filter): add autoware\_ prefix to package name (`#8183 <https://github.com/autowarefoundation/autoware.universe/issues/8183>`_)
  * chore: fix package name probabilistic occupancy grid map
  * fix: solve launch error
  * chore: update occupancy_grid_map_outlier_filter
  * style(pre-commit): autofix
  * refactor: update package name to autoware_probabilistic_occupancy_grid_map on a test
  * refactor: rename folder of occupancy_grid_map_outlier_filter
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Masaki Baba, Yoshi Ri, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
