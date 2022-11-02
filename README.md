# ArmorPy
A python client library for [**ARMOR**](https://github.com/EmaroLab/armor).

Allows users to easily call ARMOR service from Python code.

**Full documentation** at:

http://emarolab.github.io/armor_py_api/

Hides ROS service calls to ARMOR in utility functions whose structure is 
similar to that of OWL API. ARMOR is a powerful tool able to manage, query
and modify multiple ontologies in a safe way. It also include several macros
to perform complex manipulations on ARMOR references (manipulations that
usually require more than a few line of codes in OWL APIs, such as replacing
a specific data property value associated to an individual).

More functions are coming soon as their needed.

ArmorPy installs itself into the */devel/bin* of your workspace as you run
*catkin_make install*. It is then available to all your ROS Python packages.

To use it, import from 'armor_api':

    from armor_api.armor_client import ArmorClient

If your IDE cannot find the ArmorPy sources when working on a different 
project, you have to add armor_py_api/scripts/armor_api as a source folder.

In PyCharm you can add a folder as a source in the menu:

**Settings...-> Project: your_project_name-> Project Structure**.

**Note:** if your workspace is corrrectly set, installing ArmorPy is enough to
use it in your code and run it. Thus, even if your IDE cannot find your imports,
your code will work (it should find an **\_\_init\_\_.py** file only). Adding sources
will allow you to see ArmorPy code for quick reference, easier debugging and 
getting rid of annoying warnings.

**Note2:** for new versions of ross (e.g., Noetic) you might need to add in
your `.bashrc` file 
```bash
export PYTHONPATH=$PYTHONPATH:<path/to/armor_py_api>/scripts/armor_api/
```
Where `<path/to/armor_py_api>` should match with you actual installation path.
