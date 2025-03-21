^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_map_projection_loader
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(map_loader): add support for local cartesian projection (`#9238 <https://github.com/autowarefoundation/autoware_universe/issues/9238>`_)
  * feat(map_loader): add support for local cartesian projection to lanelet map loader
  * feat(map_loader): udpate readme
  * feat(map_loader): add support for local cartesian projection
  * bump autoware_msgs to 1.4.0
  ---------
  Co-authored-by: Sebastian Zęderowski <szederowski@autonomous-systems.pl>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  Co-authored-by: TaikiYamada4 <129915538+TaikiYamada4@users.noreply.github.com>
* chore(autoware_map_projection_loader): show map error details (`#10151 <https://github.com/autowarefoundation/autoware_universe/issues/10151>`_)
  * show map error details
  * use string stream
  ---------
* Contributors: Fumiya Watanabe, Sebastian Zęderowski, Yukinari Hisaki

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_component_interface_specs_universe!): rename package (`#9753 <https://github.com/autowarefoundation/autoware_universe/issues/9753>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(cpplint): include what you use - map (`#9568 <https://github.com/autowarefoundation/autoware_universe/issues/9568>`_)
* feat!: replace tier4_map_msgs with autoware_map_msgs for MapProjectorInfo (`#9392 <https://github.com/autowarefoundation/autoware_universe/issues/9392>`_)
* refactor(map_loader)!: prefix package and namespace with autoware (`#8927 <https://github.com/autowarefoundation/autoware_universe/issues/8927>`_)
  * make lanelet2_map_visualization independent
  * remove unused files
  * remove unused package
  * fix package name
  * add autoware\_ prefix
  * add autoware to exec name
  * add autoware prefix
  * removed unnecessary dependency
  ---------
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* refactor(map_loader)!: prefix package and namespace with autoware (`#8927 <https://github.com/autowarefoundation/autoware_universe/issues/8927>`_)
  * make lanelet2_map_visualization independent
  * remove unused files
  * remove unused package
  * fix package name
  * add autoware\_ prefix
  * add autoware to exec name
  * add autoware prefix
  * removed unnecessary dependency
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(component_interface_utils): prefix package and namespace with autoware (`#9092 <https://github.com/autowarefoundation/autoware_universe/issues/9092>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Masaki Baba, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(component_interface_utils): prefix package and namespace with autoware (`#9092 <https://github.com/autowarefoundation/autoware_universe/issues/9092>`_)
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(component_interface_specs): prefix package and namespace with autoware (`#9094 <https://github.com/autowarefoundation/autoware_universe/issues/9094>`_)
* refactor(map_projection_loader)!: prefix package and namespace with autoware (`#8420 <https://github.com/autowarefoundation/autoware_universe/issues/8420>`_)
  * add autoware\_ prefix
  * add autoware\_ prefix
  ---------
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* Contributors: Esteve Fernandez, Masaki Baba, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
