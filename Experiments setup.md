## Experiment Data

#### 1. Input data format

**File name**: inst_JS_200_1.csv

**Parameters:** 

| startdepot | returndepot | numOrders | radius | maxOrder | maxTime |
| ---------- | ----------- | --------- | ------ | -------- | ------- |
| B01FE15XQH | B0FFGPRVIT  | 157       | 40     | 3        | 12      |

**Orders:** 

| 属性        | 描述                 | 备注                       |
| ----------- | -------------------- | -------------------------- |
| id          | 货运订单编号         |                            |
| origin      | 货物起始地           | POI 位置，代表一个物流节点 |
| org_city    | 起始地所在城市       | 城市名称拼音               |
| destination | 货物目的地           | POI 位置，代表一个物流节点 |
| dest_city   | 目的地所在城市       | 城市名称拼音               |
| distance    | 直达驾驶距离         | 单位：km                   |
| duration    | 直达驾驶时间         | 单位：hour                 |
| pallet      | 货物数量             | 单位：pallet               |
| to          | 最早提货时刻         | 08:00, 或者为12:00         |
| td          | 交货截止时刻         | 20:00, 或者为 24:00        |
| LTL         | 零担模式单独运输费用 | 单位：CNY (人民币)         |
|             |                      |                            |

**注意：**

- 订单数量大于500以上的算例，称为大规模问题，不运行枚举算法
- 订单数量不超过500的算例，称为小规模问题，需要运行枚举算法

#### 2. Output data format

**File name**: exper_JS.csv

**Results:** 

| 数据属性   | 定义                                                         | 单位         |
| ---------- | ------------------------------------------------------------ | ------------ |
| N          | 订单数量                                                     |              |
| radius     | bundling 距离限制                                            | km           |
| maxOrder   | bundling 订单数量限制                                        |              |
| maxTime    | bundling 车辆服务时间限制                                    | hour         |
| COST_LTL   | 零担模式单独运输总成本                                       | CNY (人民币) |
|            |                                                              |              |
| ROUTE_2    | 枚举服务2个order的route数量                                  |              |
| ROUTE_3    | 枚举服务3个order的route数量                                  |              |
| ROUTE_4    | 枚举服务4个order的route数量                                  |              |
|            |                                                              |              |
| LTL_enum   | 基于枚举的精确解中单独运输的订单数量                         |              |
| MSTL2_enum | 基于枚举的精确解中服务2个order的route数量                    |              |
| MSTL3_enum | 基于枚举的精确解中服务3个order的route数量                    |              |
| MSTL4_enum | 基于枚举的精确解中服务4个order的route数量                    |              |
| COST_enum  | 基于枚举的精确解的最优成本                                   | CNY (人民币) |
| TIME_enum  | 基于枚举的精确优化算法计算时间（包含枚举路径和求解CPLEX模型的时间） | minute       |
|            |                                                              |              |
| RCG_2      | 列生成算法总共生成的服务2个order的route数量                  |              |
| RCG_3      | 列生成算法总共生成的服务3个order的route数量                  |              |
| RCG_4      | 列生成算法总共生成的服务4个order的route数量                  |              |
| LTL_CG     | 列生成算法近似解中单独运输的订单数量                         |              |
| MSTL2_CG   | 列生成算法近似解中服务2个order的route数量                    |              |
| MSTL3_CG   | 列生成算法近似解中服务3个order的route数量                    |              |
| MSTL4_CG   | 列生成算法近似解中服务4个order的route数量                    |              |
| COST_CG    | 列生成算法近似解的最优成本                                   | CNY (人民币) |
| TIME_CG    | 列生成算法计算时间                                           | minute       |
| ITER_CG    | 列生成算法迭代次数                                           |              |
|            |                                                              |              |























