from setuptools import setup

setup(name='f110_gym',
      version='0.2.1',
      author='Hongrui Zheng',
      author_email='billyzheng.bz@gmail.com',
      url='https://f1tenth.org',
      package_dir={'': 'gym'},
      install_requires=['gym==0.19.0',
		        'numpy<=1.22.0,>=1.18.0',
                        'Pillow>=9.0.1',
                        'scipy>=1.7.3',
                        'numba>=0.55.2',
                        'pyyaml>=5.3.1',
                        'pyglet<1.5',
                        'pyopengl']
      )
