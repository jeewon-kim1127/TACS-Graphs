<div align="center">
 <h1>Situational Graphs - Reasoning msgs</h1>
</div>

**Situational Graphs - Reasoning msgs** is a ROS2 package containing the messages related to the utility Reasoning, employed to generate semantic concepts in S-Graphs.  


## Messages

**Match**
- name [string]
- basis_nodes [Node[]]
- target_nodes [Node[]]
- edges [Edge[]]

**Graph**
- name [string]
- nodes [Node[]]
- edges [Edge[]]

**Node**
- id [int32]
- type [string]
- attributes [Attribute[]]

**Edge**
- origin_node [int64]
- target_node [int64]
- attributes [Attribute[]]

**Attribute**
- name [string]
- str_value [string]
- fl_value [float64[]]

