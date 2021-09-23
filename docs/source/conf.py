# Configuration file for the Sphinx documentation builder.

# -- Project information
import sphinx_rtd_theme
import sys, os
# sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('../..'))
sys.path.insert(0, os.path.abspath('../../src/frobs_rl/common/'))

project = 'FROBS_RL'
copyright = '2021, Fajardo'
author = 'Fajardo'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx_rtd_theme',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

autosummary_generate = True  # Turn on sphinx.ext.autosummary

master_doc = 'index'

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'
html_logo = 'https://plantillasdememes.com/img/plantillas/mike-wazowski-con-cara-de-sullivan01578859264.jpg'
