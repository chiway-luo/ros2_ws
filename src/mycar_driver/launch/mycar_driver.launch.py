def generate_launch_description():
    pkg_share = get_package_share_directory('mycar_driver')
    param_file = os.path.join(pkg_share, 'param', 'mycar_driver.yaml')

    return LaunchDescription([
        Node(
            package='mycar_driver',
            executable='driver',
            name='mycar_driver_cpp',
            output='screen',
            parameters=[param_file]
        )
    ])
