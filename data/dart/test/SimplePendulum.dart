<?xml version="1.0" ?>
<dart version="1.0">
    <world name="default">
        <physics>
            <time_step>0.001</time_step>
            <gravity>0 0 -9.81</gravity>
        </physics>
        
        <skeleton name="skeleton_1">
            <body name="link_1">
                <gravity>1</gravity>
                <transformation>1.57079633 0.0 0.0 0.0 0.0 10.1</transformation>
                <inertia>
                    <mass>5.0</mass>
                    <offset>0 0 -10</offset>
                    <moment_of_inertia>
                        <ixx>1e-1</ixx>
                        <iyy>1e-1</iyy>
                        <izz>1e-1</izz>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyz>0</iyz>
                    </moment_of_inertia>
                </inertia>
                <visualization_shape>
                    <transformation>0 0 0 0 0 -5.0</transformation>
                    <geometry>
                        <cylinder>
                            <radius>0.05</radius>
                            <height>10.0</height>
                        </cylinder>
                    </geometry>
                </visualization_shape>
                <collision_shape>
                    <transformation>0 0 0 0 0 -5.0</transformation>
                    <geometry>
                        <cylinder>
                            <radius>0.05</radius>
                            <height>10.0</height>
                        </cylinder>
                    </geometry>
                </collision_shape>                                
            </body>
            <body name="link_2">
                <gravity>1</gravity>
                <transformation>1.57079633 0.0 0.0 0.0 10.0 10.1</transformation>
                <inertia>
                    <mass>5.0</mass>
                    <offset>0 0 0</offset>
                    <moment_of_inertia>
                        <ixx>1e-1</ixx>
                        <iyy>1e-1</iyy>
                        <izz>1e-1</izz>
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
            <joint type="revolute" name="joint 1">
                <parent>world</parent>
                <child>link_1</child>
                <transformation>0 0 0 0 0 0</transformation>
                <axis>
                    <xyz>1 0 0</xyz>
                </axis>
            </joint>
            <joint type="revolute" name="joint 2">
                <parent>link_1</parent>
                <child>link_2</child>
                <transformation>0 0 0 0 0 0</transformation>
                <axis>
                    <xyz>0 0 1</xyz>
                </axis>
            </joint>
        </skeleton>	
        
        <skeleton name="skeleton_2">
            <body name="link_1">
                <gravity>1</gravity>
                <transformation>1.57079633 0.0 0.0 1.0 0.0 10.1</transformation>
                <inertia>
                    <mass>10.0</mass>
                    <offset>0 0 -10</offset>
                    <moment_of_inertia>
                        <ixx>1e-1</ixx>
                        <iyy>1e-1</iyy>
                        <izz>1e-1</izz>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyz>0</iyz>
                    </moment_of_inertia>
                </inertia>
                <visualization_shape>
                    <transformation>0 0 0 0 0 -5.0</transformation>
                    <geometry>
                        <cylinder>
                            <radius>0.1</radius>
                            <height>10.0</height>
                        </cylinder>
                    </geometry>
                </visualization_shape>
                <collision_shape>
                    <transformation>0 0 0 0 0 -5.0</transformation>
                    <geometry>
                        <cylinder>
                            <radius>0.1</radius>
                            <height>10.0</height>
                        </cylinder>
                    </geometry>
                </collision_shape>                                
            </body>
            <joint type="revolute" name="joint 1">
                <parent>world</parent>
                <child>link_1</child>
                <transformation>0 0 0 0 0 0</transformation>
                <axis>
                    <xyz>1 0 0</xyz>
                </axis>
            </joint>
        </skeleton>	
        
    </world>
</dart>
