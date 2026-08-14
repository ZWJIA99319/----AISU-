from setuptools import setup
name='route_executor'
setup(name=name, version='1.0.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
                  ('share/'+name,['package.xml']),('share/'+name+'/launch',['launch/execute_route.launch.py'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='maintainer',
      maintainer_email='maintainer@example.com', description='OriginCar route executor',
      license='Apache-2.0', entry_points={'console_scripts':['execute_route=route_executor.executor:main']})
