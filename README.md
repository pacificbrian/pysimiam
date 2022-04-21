To install dependencies (Mac OSX using brew):

> xcode-select install \
> brew install qt6 sphinx

To install dependencies (Ubuntu):

> apt-get install python3-pip qtwayland5 sphinx

and finally:

> pip3 install numpy pyqt6 pyqt5

optional:

> pip3 install pygame pyserial
> pip3 install hasspy-hermes-app pyserial
> pip3 install --force dataclasses-json==0.4.5

To run:

> python qtsimiam.py

Load your own supervisors and controllers into the development folders following the supervisor and controller templates.

To compile the documentation, you will need Sphinx. Get it at http://sphinx-doc.org/install.html.
After that, go into the ./docs folder and type:
 
> make html
 
If you don't have make, the following command should work as well:

> sphinx-build -b html . _build
