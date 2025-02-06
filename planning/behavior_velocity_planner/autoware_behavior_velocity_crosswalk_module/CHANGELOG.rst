^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_crosswalk_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* docs(crosswalk): fix file add miss (`#10028 <https://github.com/autowarefoundation/autoware.universe/issues/10028>`_)
* docs(crosswalk): update ttc vs ttv docs (`#10025 <https://github.com/autowarefoundation/autoware.universe/issues/10025>`_)
* feat(crosswalk): update judgle time against the stopped objects (`#9988 <https://github.com/autowarefoundation/autoware.universe/issues/9988>`_)
* chore(crosswalk): port the same direction ignore block (`#9983 <https://github.com/autowarefoundation/autoware.universe/issues/9983>`_)
* feat(crosswalk): add pass marker (`#9952 <https://github.com/autowarefoundation/autoware.universe/issues/9952>`_)
* chore(planning): move package directory for planning factor interface (`#9948 <https://github.com/autowarefoundation/autoware.universe/issues/9948>`_)
  * chore: add new package for planning factor interface
  * chore(surround_obstacle_checker): update include file
  * chore(obstacle_stop_planner): update include file
  * chore(obstacle_cruise_planner): update include file
  * chore(motion_velocity_planner): update include file
  * chore(bpp): update include file
  * chore(bvp-common): update include file
  * chore(blind_spot): update include file
  * chore(crosswalk): update include file
  * chore(detection_area): update include file
  * chore(intersection): update include file
  * chore(no_drivable_area): update include file
  * chore(no_stopping_area): update include file
  * chore(occlusion_spot): update include file
  * chore(run_out): update include file
  * chore(speed_bump): update include file
  * chore(stop_line): update include file
  * chore(template_module): update include file
  * chore(traffic_light): update include file
  * chore(vtl): update include file
  * chore(walkway): update include file
  * chore(motion_utils): remove factor interface
  ---------
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware.universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* fix: remove unnecessary parameters (`#9935 <https://github.com/autowarefoundation/autoware.universe/issues/9935>`_)
* feat(behavior_velocity_modules): add node test (`#9790 <https://github.com/autowarefoundation/autoware.universe/issues/9790>`_)
  * feat(behavior_velocity_crosswalk): add node test
  * fix
  * feat(behavior_velocity_xxx_module): add node test
  * fix
  * fix
  * fix
  * fix
  * change directory tests -> test
  ---------
* refactor(behavior_velocity_planner_common): add behavior_velocity_rtc_interface and move RTC-related implementation (`#9799 <https://github.com/autowarefoundation/autoware.universe/issues/9799>`_)
  * split into planer_common and rtc_interface
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/scene_module_interface.hpp
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_rtc_interface/include/autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  * fix
  ---------
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
* feat(behavior_velocity_planner): use XXXStamped in autoware_internal_debug_msgs (`#9744 <https://github.com/autowarefoundation/autoware.universe/issues/9744>`_)
  * feat(behavior_velocity_planner): use XXXStamped in autoware_internal_debug_msgs
  * fix
  ---------
* feat(behavior_velocity_planner): remove unnecessary tier4_api_msgs (`#9692 <https://github.com/autowarefoundation/autoware.universe/issues/9692>`_)
* Contributors: Fumiya Watanabe, Mamoru Sobue, Satoshi OTA, Takayuki Murooka, Yuki TAKAGI

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
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware.universe/issues/9570>`_)
* feat(behavior_velocity_planner)!: remove stop_reason (`#9452 <https://github.com/autowarefoundation/autoware.universe/issues/9452>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(crosswalk)!: delete wide crosswalk corresponding function (`#9329 <https://github.com/autowarefoundation/autoware.universe/issues/9329>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(crosswalk): don't use vehicle stop checker to remove unnecessary callback (`#9234 <https://github.com/autowarefoundation/autoware.universe/issues/9234>`_)
* test(crosswalk): add unit test (`#9228 <https://github.com/autowarefoundation/autoware.universe/issues/9228>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Mamoru Sobue, Ryohsuke Mitsudome, Satoshi OTA, Yuki TAKAGI, Yutaka Kondo

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
* fix(crosswalk): don't use vehicle stop checker to remove unnecessary callback (`#9234 <https://github.com/autowarefoundation/autoware.universe/issues/9234>`_)
* test(crosswalk): add unit test (`#9228 <https://github.com/autowarefoundation/autoware.universe/issues/9228>`_)
* Contributors: Esteve Fernandez, Satoshi OTA, Yuki TAKAGI, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(autoware_grid_map_utils): prefix folder structure with autoware/ (`#9170 <https://github.com/autowarefoundation/autoware.universe/issues/9170>`_)
* fix(crosswalk): fix occlusion detection range calculation and add debug markers (`#9121 <https://github.com/autowarefoundation/autoware.universe/issues/9121>`_)
* fix(crosswalk): fix passing direction calclation for the objects (`#9071 <https://github.com/autowarefoundation/autoware.universe/issues/9071>`_)
* fix(crosswalk): change exceptional handling (`#8956 <https://github.com/autowarefoundation/autoware.universe/issues/8956>`_)
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware.universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(crosswalk)!: update stop position caluculation (`#8853 <https://github.com/autowarefoundation/autoware.universe/issues/8853>`_)
* feat(crosswalk): suppress restart when the ego is close to the next stop point (`#8817 <https://github.com/autowarefoundation/autoware.universe/issues/8817>`_)
  * feat(crosswalk): suppress restart when the ego is close to the next stop point
  * update
  * add comment
  ---------
* fix(behavior_velocity_planner): align the parameters with launcher (`#8791 <https://github.com/autowarefoundation/autoware.universe/issues/8791>`_)
  parameters in behavior_velocity_planner aligned
* fix(autoware_behavior_velocity_crosswalk_module): fix unusedFunction (`#8665 <https://github.com/autowarefoundation/autoware.universe/issues/8665>`_)
  fix:unusedFunction
* fix(crosswalk): fix findEgoPassageDirectionAlongPath finding front and back point logic (`#8459 <https://github.com/autowarefoundation/autoware.universe/issues/8459>`_)
  * fix(crosswalk): fix findEgoPassageDirectionAlongPath finding front and back point logic
  * define ego_crosswalk_passage_direction later
  ---------
* fix(behavior_velocity_planner): fix cppcheck warnings of virtualCallInConstructor (`#8376 <https://github.com/autowarefoundation/autoware.universe/issues/8376>`_)
  Co-authored-by: Ryuta Kambe <ryuta.kambe@tier4.jp>
* fix(autoware_behavior_velocity_crosswalk_module): fix passedByValue (`#8210 <https://github.com/autowarefoundation/autoware.universe/issues/8210>`_)
  * fix:passedByValue
  * fix:passedByValue
  ---------
* refactor(crosswalk): clean up the structure and create a brief flowchart (`#7868 <https://github.com/autowarefoundation/autoware.universe/issues/7868>`_)
  * refactor(crosswalk): clean up the structure and create a brief flowchart
  * update
  * fix
  * static stop pose -> default stop pose
  ---------
* fix(autoware_behavior_velocity_crosswalk_module): fix shadowVariable (`#7974 <https://github.com/autowarefoundation/autoware.universe/issues/7974>`_)
  * fix:shadowVariable
  * fix:shadowVariable
  ---------
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware.universe/issues/7640>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware.universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware.universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* chore(behavior_velocity_planner): move packages (`#7526 <https://github.com/autowarefoundation/autoware.universe/issues/7526>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, Maxime CLEMENT, Mehmet Dogru, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
