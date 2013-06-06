<?xml version="1.0" ?>
<dart version="1.0">
    <world name="world 1">
        <physics>
            <time_step>0.001</time_step>
            <gravity>0 -9.81 0</gravity>
        </physics>

        <skeleton name="grount skeleton">
            <body name="ground">
                <transformation>0 0 0 0 -0.5 0</transformation>
                <visualization_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <box>
                            <size>2.0 0.01 2.0</size>
                        </box>
                    </geometry>
                </visualization_shape>
                <collision_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <box>
                            <size>2.0 0.01 2.0</size>
                        </box>
                    </geometry>
                </collision_shape>                                
            </body>
            <joint type="weld" name="joint 1">
                <parent>world</parent>
                <child>ground</child>
            </joint>
        </skeleton>

        <skeleton name="sphere skeleton 1 - no cog offset">
            <body name="sphere link 1">
                <gravity>1</gravity>
                <transformation>0 0 0 0 1.0 0</transformation>
                <inertia>
                    <mass>5</mass>
                    <offset>0 0 0</offset>
                    <moment_of_inertia>
                        <ixx>1</ixx>
                        <iyy>1</iyy>
                        <izz>1</izz>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyz>0</iyz>
                    </moment_of_inertia>
                </inertia>
                <visualization_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <ellipsoid>
                            <size>0.2 0.2 0.2</size>
                        </ellipsoid>
                    </geometry>
                </visualization_shape>
                <collision_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <ellipsoid>
                            <size>0.2 0.2 0.2</size>
                        </ellipsoid>
                    </geometry>
                </collision_shape>                                
            </body>

            <joint type="free" name="sphere free joint 1">
                <parent>world</parent>
                <child>sphere link 1</child>
            </joint>
        </skeleton>	
        
        <skeleton name="sphere skeleton - cog offset (0.05, 0.0, 0.0)">
            <body name="sphere link 1">
                <gravity>1</gravity>
                <transformation>0 0 0 0.5 1.0 0</transformation>
                <inertia>
                    <mass>5</mass>
                    <offset>0.05 0 0</offset>
                    <moment_of_inertia>
                        <ixx>1</ixx>
                        <iyy>1</iyy>
                        <izz>1</izz>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyz>0</iyz>
                    </moment_of_inertia>
                </inertia>
                <visualization_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <ellipsoid>
                            <size>0.2 0.2 0.2</size>
                        </ellipsoid>
                    </geometry>
                </visualization_shape>
                <collision_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <ellipsoid>
                            <size>0.2 0.2 0.2</size>
                        </ellipsoid>
                    </geometry>
                </collision_shape>                                
            </body>

            <joint type="free" name="sphere free joint 1">
                <parent>world</parent>
                <child>sphere link 1</child>
            </joint>
        </skeleton>	
        
        
    </world>
</dart>
