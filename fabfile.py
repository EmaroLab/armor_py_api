from fabric.api import *
from fabric.contrib.console import confirm

def create_sphinx_pages():
    """
    Create a new branch with Sphinx documentation ready to be published
    using GitHub's Pages system.

    Example usage:

        $ fab create_sphinx_pages

    """    

    # Create the new branch
    local("git branch gh-pages")
    # Move into it
    local("git checkout gh-pages")
    # Clear it out
    local("git symbolic-ref HEAD refs/heads/gh-pages")
    local("rm .git/index")
    local("git clean -fdx")
    # Start up a Sphinx project, reply y to 2nd question
    # all other questions can be left unchanged
    local("sphinx-quickstart")
    # Create nojekyll file for github
    local("touch .nojekyll")
    # Make the patches to Sphinx's Makefile we need
    local("echo '' >> Makefile")
    local("echo 'BUILDDIR      = ./' >> Makefile")
    local("echo '' >> Makefile")
    local("echo 'html:' >> Makefile")
    local("echo '\t$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(BUILDDIR)' >> Makefile")
    local("echo '\t@echo' >> Makefile")
    local("echo '\t@echo \"Build finished. The HTML pages are in $(BUILDDIR)\"' >> Makefile")
    # Make the branch for the first time
    local("make html")