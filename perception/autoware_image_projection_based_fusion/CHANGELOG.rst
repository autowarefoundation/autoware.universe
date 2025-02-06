^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_image_projection_based_fusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_image_projection_based_fusion)!: tier4_debug-msgs changed to autoware_internal_debug_msgs in autoware_image_projection_based_fusion (`#9877 <https://github.com/autowarefoundation/autoware.universe/issues/9877>`_)
* fix(image_projection_based_fusion):  revise message publishers (`#9865 <https://github.com/autowarefoundation/autoware.universe/issues/9865>`_)
  * refactor: fix condition for publishing painted pointcloud message
  * fix: publish output revised
  * feat: fix condition for publishing painted pointcloud message
  * feat: roi-pointclout  fusion - publish empty image even when there is no target roi
  * fix: remap output topic for clusters in roi_pointcloud_fusion
  * style(pre-commit): autofix
  * feat: fix condition for publishing painted pointcloud message
  * feat: Add debug publisher for internal debugging
  * feat: remove !! pointer to bool conversion
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_tensorrt_common): multi-TensorRT compatibility & tensorrt_common as unified lib for all perception components (`#9762 <https://github.com/autowarefoundation/autoware.universe/issues/9762>`_)
  * refactor(autoware_tensorrt_common): multi-TensorRT compatibility & tensorrt_common as unified lib for all perception components
  * style(pre-commit): autofix
  * style(autoware_tensorrt_common): linting
  * style(autoware_lidar_centerpoint): typo
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * docs(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * fix(autoware_lidar_transfusion): reuse cast variable
  * fix(autoware_tensorrt_common): remove deprecated inference API
  * style(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * style(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * fix(autoware_tensorrt_common): const pointer
  * fix(autoware_tensorrt_common): remove unused method declaration
  * style(pre-commit): autofix
  * refactor(autoware_tensorrt_common): readability
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * fix(autoware_tensorrt_common): return if layer not registered
  * refactor(autoware_tensorrt_common): readability
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * fix(autoware_tensorrt_common): rename struct
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* fix(image_projection_based_fusion): remove mutex (`#9862 <https://github.com/autowarefoundation/autoware.universe/issues/9862>`_)
  refactor: Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
* refactor(autoware_image_projection_based_fusion): organize 2d-detection related members (`#9789 <https://github.com/autowarefoundation/autoware.universe/issues/9789>`_)
  * chore: input_camera_topics\_ is only for debug
  * feat: fuse main message with cached roi messages in fusion_node.cpp
  * chore: add comments on each process step, organize methods
  * feat: Export process method in fusion_node.cpp
  Export the `exportProcess()` method in `fusion_node.cpp` to handle the post-processing and publishing of the fused messages. This method cancels the timer, performs the necessary post-processing steps, publishes the output message, and resets the flags. It also adds processing time for debugging purposes. This change improves the organization and readability of the code.
  * feat: Refactor fusion_node.hpp and fusion_node.cpp
  Refactor the `fusion_node.hpp` and `fusion_node.cpp` files to improve code organization and readability. This includes exporting the `exportProcess()` method in `fusion_node.cpp` to handle post-processing and publishing of fused messages, adding comments on each process step, organizing methods, and fusing the main message with cached ROI messages. These changes enhance the overall quality of the codebase.
  * Refactor fusion_node.cpp and fusion_node.hpp for improved code organization and readability
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * feat: Refactor fusion_node.cpp for improved code organization and readability
  * Refactor fusion_node.cpp for improved code organization and readability
  * feat: implement mutex per 2d detection process
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * revise template, inputs first and output at the last
  * explicit in and out types 1
  * clarify pointcloud message type
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * Refactor publisher types in fusion_node.hpp and node.hpp
  * fix: resolve cppcheck issue shadowVariable
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * chore: rename Det2dManager to Det2dStatus
  * revert mutex related changes
  * refactor: review member and method's access
  * fix: resolve shadowVariable of 'det2d'
  * fix missing line
  * refactor message postprocess and publish methods
  * publish the main message is common
  * fix: replace pointcloud message type by the typename
  * review member access
  * Refactor fusion_node.hpp and fusion_node.cpp for improved code organization and readability
  * refactor: fix condition for publishing painted pointcloud message
  * fix: remove unused variable
  ---------
* feat(lidar_centerpoint, pointpainting): add diag publisher for max voxel size (`#9720 <https://github.com/autowarefoundation/autoware.universe/issues/9720>`_)
* feat(pointpainting_fusion): enable cloud display on image (`#9813 <https://github.com/autowarefoundation/autoware.universe/issues/9813>`_)
* feat(image_projection_based_fusion): add cache for camera projection (`#9635 <https://github.com/autowarefoundation/autoware.universe/issues/9635>`_)
  * add camera_projection class and projection cache
  * style(pre-commit): autofix
  * fix FOV filtering
  * style(pre-commit): autofix
  * remove unused includes
  * update schema
  * fix cpplint error
  * apply suggestion: more simple and effcient computation
  * correct terminology
  * style(pre-commit): autofix
  * fix comments
  * fix var name
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * fix variable names
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * fix variable names
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * fix variable names
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * fix variable names
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * fix variable names
  * fix initialization
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * add verification to point_project_to_unrectified_image when loading
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * chore: add option description to project points to unrectified image
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* feat(image_projection_based_fusion): add timekeeper (`#9632 <https://github.com/autowarefoundation/autoware.universe/issues/9632>`_)
  * add timekeeper
  * chore: refactor time-keeper position
  * chore: bring back a missing comment
  * chore: remove redundant timekeepers
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Amadeusz Szymko, Fumiya Watanabe, Masaki Baba, Taekjin LEE, Vishal Chauhan, Yi-Hsiang Fang (Vivid), kminoda

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware.universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix(lidar_centerpoint): non-maximum suppression target decision logic (`#9595 <https://github.com/autowarefoundation/autoware.universe/issues/9595>`_)
  * refactor(lidar_centerpoint): optimize non-maximum suppression search distance calculation
  * feat(lidar_centerpoint): do not suppress if one side of the object is pedestrian
  * style(pre-commit): autofix
  * refactor(lidar_centerpoint): remove unused variables
  * refactor: remove unused variables
  fix: implement non-maximum suppression logic to the transfusion
  refactor: remove unused parameter iou_nms_target_class_names
  Revert "fix: implement non-maximum suppression logic to the transfusion"
  This reverts commit b8017fc366ec7d67234445ef5869f8beca9b6f45.
  fix: revert transfusion modification
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: remove max rois limit in the image projection based fusion (`#9596 <https://github.com/autowarefoundation/autoware.universe/issues/9596>`_)
  feat: remove max rois limit
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware.universe/issues/9588>`_)
* fix(autoware_image_projection_based_fusion): detected object roi box projection fix (`#9519 <https://github.com/autowarefoundation/autoware.universe/issues/9519>`_)
  * fix: detected object roi box projection fix
  1. eliminate misuse of std::numeric_limits<double>::min()
  2. fix roi range up to the image edges
  * fix: fix roi range calculation in RoiDetectedObjectFusionNode
  Improve the calculation of the region of interest (ROI) in the RoiDetectedObjectFusionNode. The previous code had an issue where the ROI range was not correctly limited to the image edges. This fix ensures that the ROI is within the image boundaries by using the correct comparison operators for the x and y coordinates.
  ---------
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware.universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware.universe/issues/9588>`_)
* ci(pre-commit): update cpplint to 2.0.0 (`#9557 <https://github.com/autowarefoundation/autoware.universe/issues/9557>`_)
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware.universe/issues/9569>`_)
* chore(image_projection_based_fusion): add debug for roi_pointcloud fusion (`#9481 <https://github.com/autowarefoundation/autoware.universe/issues/9481>`_)
* fix(autoware_image_projection_based_fusion): fix clang-diagnostic-inconsistent-missing-override (`#9509 <https://github.com/autowarefoundation/autoware.universe/issues/9509>`_)
* fix(autoware_image_projection_based_fusion): fix clang-diagnostic-unused-private-field (`#9505 <https://github.com/autowarefoundation/autoware.universe/issues/9505>`_)
* fix(autoware_image_projection_based_fusion): fix clang-diagnostic-inconsistent-missing-override (`#9495 <https://github.com/autowarefoundation/autoware.universe/issues/9495>`_)
* fix(autoware_image_projection_based_fusion): fix clang-diagnostic-inconsistent-missing-override (`#9516 <https://github.com/autowarefoundation/autoware.universe/issues/9516>`_)
  fix: clang-diagnostic-inconsistent-missing-override
* fix(autoware_image_projection_based_fusion): fix clang-diagnostic-inconsistent-missing-override (`#9510 <https://github.com/autowarefoundation/autoware.universe/issues/9510>`_)
* fix(autoware_image_projection_based_fusion): fix clang-diagnostic-unused-private-field (`#9473 <https://github.com/autowarefoundation/autoware.universe/issues/9473>`_)
  * fix: clang-diagnostic-unused-private-field
  * fix: build error
  ---------
* fix(autoware_image_projection_based_fusion): fix clang-diagnostic-inconsistent-missing-override (`#9472 <https://github.com/autowarefoundation/autoware.universe/issues/9472>`_)
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
* fix(autoware_image_projection_based_fusion): make optional to consider lens distortion in the point projection (`#9233 <https://github.com/autowarefoundation/autoware.universe/issues/9233>`_)
  chore: add point_project_to_unrectified_image parameter to fusion_common.param.yaml
* fix(autoware_image_projection_based_fusion): fix bugprone-misplaced-widening-cast (`#9226 <https://github.com/autowarefoundation/autoware.universe/issues/9226>`_)
  * fix: bugprone-misplaced-widening-cast
  * fix: clang-format
  ---------
* fix(autoware_image_projection_based_fusion): fix bugprone-misplaced-widening-cast (`#9229 <https://github.com/autowarefoundation/autoware.universe/issues/9229>`_)
  * fix: bugprone-misplaced-widening-cast
  * fix: clang-format
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Yoshi Ri, Yutaka Kondo, awf-autoware-bot[bot], badai nguyen, kobayu858

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
* fix(autoware_image_projection_based_fusion): make optional to consider lens distortion in the point projection (`#9233 <https://github.com/autowarefoundation/autoware.universe/issues/9233>`_)
  chore: add point_project_to_unrectified_image parameter to fusion_common.param.yaml
* fix(autoware_image_projection_based_fusion): fix bugprone-misplaced-widening-cast (`#9226 <https://github.com/autowarefoundation/autoware.universe/issues/9226>`_)
  * fix: bugprone-misplaced-widening-cast
  * fix: clang-format
  ---------
* fix(autoware_image_projection_based_fusion): fix bugprone-misplaced-widening-cast (`#9229 <https://github.com/autowarefoundation/autoware.universe/issues/9229>`_)
  * fix: bugprone-misplaced-widening-cast
  * fix: clang-format
  ---------
* Contributors: Esteve Fernandez, Taekjin LEE, Yutaka Kondo, kobayu858

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(autoware_point_types): prefix namespace with autoware::point_types (`#9169 <https://github.com/autowarefoundation/autoware.universe/issues/9169>`_)
* fix(autoware_image_projection_based_fusion): pointpainting bug fix for point projection (`#9150 <https://github.com/autowarefoundation/autoware.universe/issues/9150>`_)
  fix: projected 2d point has 1.0 of depth
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware.universe/issues/8946>`_)
* fix(autoware_image_projection_based_fusion): roi cluster fusion has no existence probability update (`#8864 <https://github.com/autowarefoundation/autoware.universe/issues/8864>`_)
  fix: add existence probability update, refactoring
* fix(autoware_image_projection_based_fusion): resolve issue with segmentation pointcloud fusion node failing with multiple mask inputs (`#8769 <https://github.com/autowarefoundation/autoware.universe/issues/8769>`_)
* fix(image_projection_based_fusion): remove unused variable (`#8634 <https://github.com/autowarefoundation/autoware.universe/issues/8634>`_)
  fix: remove unused variable
* fix(autoware_image_projection_based_fusion): fix unusedFunction (`#8567 <https://github.com/autowarefoundation/autoware.universe/issues/8567>`_)
  fix:unusedFunction
* fix(image_projection_based_fusion): add run length decoding for segmentation_pointcloud_fusion (`#7909 <https://github.com/autowarefoundation/autoware.universe/issues/7909>`_)
  * fix: add rle decompress
  * style(pre-commit): autofix
  * fix: use rld in tensorrt utils
  * fix: rebase error
  * fix: dependency
  * fix: skip publish debug mask
  * Update perception/autoware_image_projection_based_fusion/src/segmentation_pointcloud_fusion/node.cpp
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  * style(pre-commit): autofix
  * Revert "fix: skip publish debug mask"
  This reverts commit 30fa9aed866a019705abde71e8f5c3f98960c19e.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* fix(image_projection_based_fusion): handle projection errors in image fusion nodes (`#7747 <https://github.com/autowarefoundation/autoware.universe/issues/7747>`_)
  * fix: add check for camera distortion model
  * feat(utils): add const qualifier to local variables in checkCameraInfo function
  * style(pre-commit): autofix
  * chore(utils): update checkCameraInfo function to use RCLCPP_ERROR_STREAM for unsupported distortion model and coefficients size
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_image_projection_based_fusion): fix passedByValue (`#8234 <https://github.com/autowarefoundation/autoware.universe/issues/8234>`_)
  fix:passedByValue
* refactor(image_projection_based_fusion)!: add package name prefix of autoware\_ (`#8162 <https://github.com/autowarefoundation/autoware.universe/issues/8162>`_)
  refactor: rename image_projection_based_fusion to autoware_image_projection_based_fusion
* Contributors: Esteve Fernandez, Taekjin LEE, Yi-Hsiang Fang (Vivid), Yoshi Ri, Yutaka Kondo, badai nguyen, kobayu858

0.26.0 (2024-04-03)
-------------------
