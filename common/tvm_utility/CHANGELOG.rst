^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tvm_utility
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(tvm_utility): add virtual destructor to tvm_utility (`#7759 <https://github.com/youtalk/autoware.universe/issues/7759>`_)
  * fix(tvm_utility): add virtual destructor to tvm_utility
  * add override
  ---------
* fix(tvm_utility): fix selfAssignment warnings (`#7561 <https://github.com/youtalk/autoware.universe/issues/7561>`_)
* fix(tvm_utility): fix warning of negativeContainerIndex (`#6924 <https://github.com/youtalk/autoware.universe/issues/6924>`_)
* Contributors: Ryuta Kambe, Yukinari Hisaki, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* build(tvm_utility): remove download logic from CMake and update documentation (`#4923 <https://github.com/youtalk/autoware.universe/issues/4923>`_)
  * add include tier4_autoware_utils and dependency
  * remove downloading logic from Cmake, update documentation
  * build(tvm_utility): remove downloading logic from Cmake, update documentation
  * style(pre-commit): autofix
  * build(tvm_utility): fix lint_cmake error
  * build(tvm_utility): format warning message
  * build(tvm_utility): add logic to work with autoware_data folder, add nn config header and test image
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * build(tvm_utility): refactor, update InferenceEngineTVM constructor
  * style(pre-commit): autofix
  * build(tvm_utility): add lightweight model and test with it
  * build(tvm_utility): make building yolo_v2_tiny disable by default
  * build(tvm_utility): remove test artifact for yolo_v2_tiny
  * build(tvm_utility): update docs
  * build(tvm_utility): update docs
  * style(pre-commit): autofix
  * build(tvm_utility): update namespace in abs_model test
  * build(tvm_utility): rewrite yolo_v2_tiny as example
  * build(tvm_utility): clean comments in yolo_v2_tiny example main.cpp
  * build(tvm_utility): add launch file for yolo_v2_tiny example
  * build(tvm_utility): update yolo_v2_tiny example readme
  * style(pre-commit): autofix
  * build(tvm_utility): add model for arm based systems, need to be tested on device
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * build(tvm_utility): update config header for arm
  * style(pre-commit): autofix
  * build(tvm_utility): remove debug output
  * build(tvm_utility): add find_package conditional section
  * build(tvm_utility): fix lint_cmake errors
  * build(tvm_utility): remove coping model files during build
  * build(tvm_utility): update readme with new data folder structure
  * build(tvm_utility): fix spell check warnings
  * style(pre-commit): autofix
  * build(tvm_utility): add no model files guard to get_neural_network
  * style(pre-commit): autofix
  * build(tvm_utility): set back default paths in config headers
  * build(tvm_utility): add param file, update launch file
  * build(tvm_utility): add schema file, update node name
  * style(pre-commit): autofix
  * build(tvm_utility): fix json-schema-check
  * build(tvm_utility): fix json-schema-check
  * style(pre-commit): autofix
  * build(tvm_utility): add parameter table to example readme
  * build(tvm_utility): fix typo-error in description of schema.json
  * style(pre-commit): autofix
  * buiild(tvm_utility): fix spell-check warning and typo
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor: fix spell-check (`#4289 <https://github.com/youtalk/autoware.universe/issues/4289>`_)
  * refactor: fix spell-check
  * fix spell-check
  * fix typo
  * Fix obvious typo, shortened words
  * Fix obvious typo, shortened words, in common directory
  * add cspell ignore
  * Update perception/tensorrt_classifier/CMakeLists.txt
  Co-authored-by: Yusuke Muramatsu <yukke42@users.noreply.github.com>
  ---------
  Co-authored-by: Yusuke Muramatsu <yukke42@users.noreply.github.com>
* build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/youtalk/autoware.universe/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
* refactor(lidar_apollo_segmentation_tvm/_nodes): remove autoware_auto_common dependency (`#3136 <https://github.com/youtalk/autoware.universe/issues/3136>`_)
* docs: comply with mkdocs style (`#3378 <https://github.com/youtalk/autoware.universe/issues/3378>`_)
  refactor(doc): comply with mkdocs style
  Change from the Autoware.Auto style of package documentation to one that
  gets parsed better by the current way of generating the documentation.
  Issue-Id: SCM-5887
  Change-Id: I898d00eda921ac73373067036c582a7bbc192cce
  Co-authored-by: Xinyu Wang <93699235+angry-crab@users.noreply.github.com>
* feat(tvm_utility, lidar_centerpoint_tvm): tvm pipeline extension (`#2468 <https://github.com/youtalk/autoware.universe/issues/2468>`_)
  * add scatter
  * two stage pipeline
  * ci(pre-commit): autofix
  * fix build
  * ci(pre-commit): autofix
  * remove unecessary code
  * apply changes from network_node refactor
  * ci(pre-commit): autofix
  * unpack tar file first and add maintainer
  * fix pre commit error
  * remove comment
  * fix cmake download, remove scatter cpp and rename tvm function
  * ci(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(tvm_utility): redefine network node and inference config (`#2467 <https://github.com/youtalk/autoware.universe/issues/2467>`_)
  * redefine network node and inference config
  * accomondate network node changes
  * ci(pre-commit): autofix
  * revert device type and id
  * cmake setting
  * data type and modelzoo version
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(tvm_utility): copy test result to CPU (`#2414 <https://github.com/youtalk/autoware.universe/issues/2414>`_)
  Also remove dependency to autoware_auto_common.
  Issue-Id: SCM-5401
  Change-Id: I83b859742df2f2ff7df1d0bd2d287bfe0aa04c3d
  Co-authored-by: Xinyu Wang <93699235+angry-crab@users.noreply.github.com>
* fix(lidar_apollo_segmentation_tvm, tvm_utility): fixed output dim and pipeline (`#2185 <https://github.com/youtalk/autoware.universe/issues/2185>`_)
* feat(ModelZoo): model descriptors versioning (`#2158 <https://github.com/youtalk/autoware.universe/issues/2158>`_)
  Introduce a version field to the interface.
  Make use of it in the lidar_apollo_segmentation_tvm packages and the
  yolo_v2 example of tvm_utility.
  Issue-Id: SCM-4000
  Change-Id: I6d666f886b0a9f01128bfa4cf30e189d23f3481e
  Co-authored-by: Xinyu Wang <93699235+angry-crab@users.noreply.github.com>
* feat(ModelZoo): rework prebuilt assets management (`#1880 <https://github.com/youtalk/autoware.universe/issues/1880>`_)
  ModelZoo artifacts changed from a single archive to one archive per
  model/backend combination. It allows users to only download needed
  archives, but prevents keeping the current design.
  Change from all models being handled by the "neural_networks_provider"
  package to models being downloaded by packages that need them.
  Leverage the newly added versioning of the prebuilt models.
  Fix the NN check of the "nodes" package.
  Issue-Id: SCM-3999
  Change-Id: I1df9007f5bf446a8b50e38c4fd98e9e3a8d2550f
  Co-authored-by: Xinyu Wang <93699235+angry-crab@users.noreply.github.com>
* docs(tvm_utility): fix broken link (`#1670 <https://github.com/youtalk/autoware.universe/issues/1670>`_)
* feat(tvm_utility): port tvm_utility (`#893 <https://github.com/youtalk/autoware.universe/issues/893>`_)
  Co-authored-by: Luca Foschiani <luca.foschiani@arm.com>
* Contributors: Alexey Panferov, Ambroise Vincent, Kenji Miyake, Luca Foschiani, M. Fatih Cırıt, Shunsuke Miura, Vincent Richard, Xinyu Wang
