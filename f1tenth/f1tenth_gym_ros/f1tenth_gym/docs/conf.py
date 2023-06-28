import os
import sys
import sphinx_rtd_theme

source_suffix = '.rst'
source_encoding = 'utf-8-sig'

# -- Language ----------------------------------------------------------------
env_tags = os.getenv('SPHINX_TAGS')
if env_tags != None:
    for tag in env_tags.split(','):
        print('Adding Sphinx tag: %s' % tag.strip())
        tags.add(tag.strip())

language = os.getenv('READTHEDOCS_LANGUAGE', 'en')
is_i18n = tags.has('i18n')

# -- Theme -------------------------------------------------------------------
on_rtd = os.environ.get('READTHEDOCS', None) == 'True'
html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
if on_rtd:
    using_rtd_theme = True
html_theme_options = {
    # 'typekit_id': 'hiw1hhg',
    # 'analytics_id': '',
    # 'sticky_navigation': True  # Set to False to disable the sticky nav while scrolling.
    'logo_only': False,  # if we have a html_logo below, this shows /only/ the logo with no title text
    'collapse_navigation': False,  # Collapse navigation (False makes it tree-like)
    'prev_next_buttons_location': 'bottom',
    # 'display_version': True,  # Display the docs version
    # 'navigation_depth': 4,  # Depth of the headers shown in the navigation bar
}
html_context = {
    "display_github": not is_i18n, # Integrate GitHub
    "github_user": "f1tenth", # Username
    "github_repo": "f1tenth_gym", # Repo name
    "github_version": "exp_py", # Version
    "conf_py_path": "/docs/", # Path in the checkout to the docs root
}

html_favicon = 'assets/f1_stickers_02.png'

html_css_files = [
    'css/custom.css'
]

html_js_files = [
    'css/custom.js'
]
html_logo = 'assets/f1tenth_gym.svg'

# -- Project information -----------------------------------------------------

project = 'f1tenth_gym'
copyright = "2021, Hongrui Zheng, Matthew O'Kelly, Aman Sinha"
author = 'Hongrui Zheng'

# The full version, including alpha/beta/rc tags
release = 'latest'
version = 'latest'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [ "breathe", "sphinx_rtd_theme", "sphinx.ext.autosectionlabel"
]

# Breathe configuration
breathe_projects = {
        "f1tenth_gym":"./xml"
        }

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']