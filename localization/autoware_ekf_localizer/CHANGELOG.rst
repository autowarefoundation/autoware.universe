^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ekf_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* revert: revert "feat(autoware_ekf_localizer)!: porting from universe to core (`#9978 <https://github.com/autowarefoundation/autoware.universe/issues/9978>`_)" (`#10004 <https://github.com/autowarefoundation/autoware.universe/issues/10004>`_)
  This reverts commit 037c315fbee69bb5923ec10bb8e8e70f890725ea.
* feat(autoware_ekf_localizer)!: porting from universe to core (`#9978 <https://github.com/autowarefoundation/autoware.universe/issues/9978>`_)
  * feat: delete ekf_localizer files
  * doc: Modify ekf_localizer directory links
  * ci: remove ekf_localizer from the codecov target list
  ---------
* feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in fies localization/autoware_ekf_localizer (`#9860 <https://github.com/autowarefoundation/autoware.universe/issues/9860>`_)
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* feat(ekf_localizer): check whether the initialpose has been set (`#9787 <https://github.com/autowarefoundation/autoware.universe/issues/9787>`_)
  * check set intialpose
  * update png
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Motz, Ryohsuke Mitsudome, Vishal Chauhan, Yamato Ando

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
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
* fix(cpplint): include what you use - localization (`#9567 <https://github.com/autowarefoundation/autoware.universe/issues/9567>`_)
* fix(autoware_ekf_localizer): publish `processing_time_ms` (`#9443 <https://github.com/autowarefoundation/autoware.universe/issues/9443>`_)
  Fixed to publish processing_time_ms
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
* fix(autoware_ekf_localizer): remove `timer_tf\_` (`#9244 <https://github.com/autowarefoundation/autoware.universe/issues/9244>`_)
  Removed timer_tf\_
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, SakodaShintaro, Yutaka Kondo

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
* fix(autoware_ekf_localizer): remove `timer_tf\_` (`#9244 <https://github.com/autowarefoundation/autoware.universe/issues/9244>`_)
  Removed timer_tf\_
* Contributors: Esteve Fernandez, SakodaShintaro, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(localization_util)!: prefix package and namespace with autoware (`#8922 <https://github.com/autowarefoundation/autoware.universe/issues/8922>`_)
  add autoware prefix to localization_util
* refactor(ekf_localizer)!: prefix package and namespace with autoware (`#8888 <https://github.com/autowarefoundation/autoware.universe/issues/8888>`_)
  * import lanelet2_map_preprocessor
  * move headers to include/autoware/efk_localier
  ---------
* Contributors: Masaki Baba, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
