# モーションユーティリティパッケージ

## 用語の定義

### セグメント

Autowareにおける`セグメント`は、次のような2つの連続したポイント間の線分です。

![segment](./media/segment.svg){: style="width:600px"}

特定の位置に対する最近接セグメントインデックスと最近接ポイントインデックスは、常に同じではありません。
そのため、ポイントとセグメントの最近接インデックスを計算するための2つの異なるユーティリティ関数を用意しています。

## 最近接インデックス検索

このセクションでは、最近接インデックス検索と最近接セグメントインデックス検索について説明します。

最近接インデックス検索と最近接セグメントインデックス検索には同じ関数を使用します。
最も近いインデックス検索の例を挙げると、2種類の関数があります。

最初の関数は、距離とヨーのしきい値を使用して最近接インデックスを見つけるものです。


```cpp
template <class T>
size_t findFirstNearestIndexWithSoftConstraints(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());
```

この関数はしきい値内の最初のローカルソリューションを求めます。最初のローカルソリューションを求める理由は、次の小節で説明するエッジケースを処理するためです。

しきい値引数にはデフォルトのパラメータがあるので、関数の呼び出し時にどのしきい値を渡すかを決定できます。

1. 距離とヨーの両方のしきい値が指定されている場合
   - まず、距離とヨーの両方のしきい値を持つ最も近いインデックスを見つけようとします。
   - 見つからない場合、距離のしきい値のみで再度見つけようとします。
   - 見つからない場合、しきい値なしで見つけます。
2. 距離のみが指定されている場合
   - まず、距離のしきい値を持つ最も近いインデックスを見つけようとします。
   - 見つからない場合、しきい値なしで見つけます。
3. しきい値が指定されていない場合
   - 最も近いインデックスを見つけます。

2 番目の関数は、`lane_id`であるレーンの最も近いインデックスを見付けます。


```cpp
size_t findNearestIndexFromLaneId(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & pos, const int64_t lane_id);
```

### さまざまなオブジェクトへの適用

多くのノードパッケージは、オブジェクトの最近接インデックスを計算することがよくあります。
推奨される計算方法について説明します。

#### 自車に対する最近接インデックス

自車の前方のパス長が十分に短い場合、距離とヨー角の両方の閾値を使用して`findFirstNearestIndexWithSoftConstraints`を使用して、次のエッジケースで正しい最近接インデックスが得られることが期待されます。
青い円は、ベースリンク位置からの距離閾値を表し、2つの青い線はベースリンクの向きに対してヨー角の閾値を表します。
これらのケース内の点の中で、赤い正しい最近接点が検出できます。

![ego_nearest_search](./media/ego_nearest_search.svg)

したがって、実装は以下のようになります。


```cpp
const size_t ego_nearest_idx = findFirstNearestIndexWithSoftConstraints(points, ego_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
const size_t ego_nearest_seg_idx = findFirstNearestIndexWithSoftConstraints(points, ego_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
```

#### 動的オブジェクトの nearest index

エゴの nearest index では、エゴが点に従うことが想定されるため、位置に加えて向きも考慮されます。
ただし、動的オブジェクト (たとえば、予測オブジェクト) の場合、動的オブジェクトが後退していても、エゴが前進していても、オブジェクトの向きが点の順序と異なる場合があります。

したがって、動的オブジェクトではヨー閾値は考慮されるべきではありません。
実装は次のようになります。


```cpp
const size_t dynamic_obj_nearest_idx = findFirstNearestIndexWithSoftConstraints(points, dynamic_obj_pose, dynamic_obj_nearest_dist_threshold);
const size_t dynamic_obj_nearest_seg_idx = findFirstNearestIndexWithSoftConstraints(points, dynamic_obj_pose, dynamic_obj_nearest_dist_threshold);
```

#### 交通対象の最近インデックス

Laneletマップでは、交通対象は特定のレーンに属しています。
この特定のレーンのIDにより、正しい最近インデックスを検索できます。

実装方法は次のとおりです。


```cpp
// first extract `lane_id` which the traffic object belong to.
const size_t traffic_obj_nearest_idx = findNearestIndexFromLaneId(path_with_lane_id, traffic_obj_pos, lane_id);
const size_t traffic_obj_nearest_seg_idx = findNearestSegmentIndexFromLaneId(path_with_lane_id, traffic_obj_pos, lane_id);
```

## 開発者向け

`trajectory.hpp` の一部テンプレート関数は、主に特定の型 (`autoware_planning_msgs::msg::PathPoint`, `autoware_planning_msgs::msg::PathPoint`, `autoware_planning_msgs::msg::TrajectoryPoint`) で使用されるため、コンパイル時間を短縮するために `extern template` 関数としてエクスポートされています。

`autoware_motion_utils.hpp` ヘッダーファイルは、このファイルを直接/間接的に含むソースファイルがプリプロセスに時間がかかったため削除されました。

