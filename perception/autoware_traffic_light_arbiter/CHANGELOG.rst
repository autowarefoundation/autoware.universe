^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_arbiter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware.universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* feat(autoware_traffic_light_arbiter): added schema and related files for autoware_traffic_light_arbiter (`#10100 <https://github.com/autowarefoundation/autoware.universe/issues/10100>`_)
  * Added schema and related files for autoware_traffic_light_arbiter
  * Added traffic_light_arbiter.schema.json
  * style(pre-commit): autofix
  * fix
  * for perform ci
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
* Contributors: Fumiya Watanabe, Shunsuke Miura, Vishal Chauhan

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware.universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_traffic_light_arbiter): add current time validation (`#9747 <https://github.com/autowarefoundation/autoware.universe/issues/9747>`_)
  * add current time validation
  * style(pre-commit): autofix
  * change ros parameter name
  * style(pre-commit): autofix
  * add validation with absolute function
  * add timestamp of topic in test
  * fix ci error
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Masato Saeki

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
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware.universe/issues/9569>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/autowarefoundation/autoware.universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Masato Saeki, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/autowarefoundation/autoware.universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Masato Saeki, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(autoware_traffic_light_arbiter): fix build error (`#9186 <https://github.com/autowarefoundation/autoware.universe/issues/9186>`_)
  fix build error
* test(autoware_traffic_light_arbiter): add node test (`#8747 <https://github.com/autowarefoundation/autoware.universe/issues/8747>`_)
  * add test dir
  * update test node
  * style(pre-commit): autofix
  * refactor
  * style(pre-commit): autofix
  * add std namespace to size_t
  * fix typo
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(traffic_light_arbiter): missing name changes (`#8278 <https://github.com/autowarefoundation/autoware.universe/issues/8278>`_)
  chore: missing name changes
* refactor: traffic light arbiter/autoware prefix (`#8181 <https://github.com/autowarefoundation/autoware.universe/issues/8181>`_)
  * refactor(traffic_light_arbiter): apply `autoware` namespace to traffic_light_arbiter
  * refactor(traffic_light_arbiter): update the package name in CODEWONER
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Go Sakayori, Kenzo Lobos Tsunekawa, Manato Hirabayashi, Masato Saeki, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
