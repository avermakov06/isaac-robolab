from setuptools import find_packages, setup


with open('requirements.txt') as f:
    requirements = f.read().splitlines()


setup(
    name='API',
    version='1.3.7',
    description=(
        'Python programming interface for controlling RC series '
        'collaborative robots.'
    ),
    packages=find_packages(include=['API', 'API.*']),
    install_requires=requirements,
    python_requires='>=3.10',
    author_email='ryabov.pavel@rozum.com',
    zip_safe=False
)
