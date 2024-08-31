import ipywidgets as widgets
from ipywidgets import HBox, Button, Output
from IPython.display import display, clear_output, HTML

from pycram.context_knowledge import ContextConfig, generate_context

contexts = [('Select', None),('Breakfast', "breakfast"), ('Clean Up', "clean_up")]
environments = [('Select', None), ('Apartment', "apartment-small.urdf"), ('Kitchen (Unavailable)', None)] #('Kitchen', "kitchen-small.urdf")
locations = [('Select', None), ('Table', "table_area_main"), ('Countertop', "island_countertop")]

selected_context = None
selected_environment = None
selected_location = None


def update_globals(context=None, environment=None, location=None):
    global selected_context, selected_environment, selected_location
    if context is not None:
        selected_context = context
    if environment is not None:
        selected_environment = environment
    if location is not None:
        selected_location = location


def robot_execute(func):
    global selected_context, selected_environment, selected_location
    with output:
        output.clear_output()
        func(selected_location, selected_context, selected_environment, "iai_donbot")



def setup_task_object_widgets():
    context_dropdown = widgets.Dropdown(options=contexts, description='Context:')
    environment_dropdown = widgets.Dropdown(options=environments, description='Environment:')
    location_dropdown = widgets.Dropdown(options=locations, description='Target:')

    context_dropdown.observe(lambda change: update_globals(context=change['new']), names='value')
    environment_dropdown.observe(lambda change: update_globals(environment=change['new']), names='value')
    location_dropdown.observe(lambda change: update_globals(location=change['new']), names='value')

    display(HBox([context_dropdown, environment_dropdown, location_dropdown]))


def start_demo(func):
    global output
    output = Output()
    setup_task_object_widgets()
    execute_button = Button(description="Execute Task")
    # Use a lambda function to defer the call to `robot_execute`
    # In this lambda function, lambda x: robot_execute(func),
    # x represents the button click event (which we don't use here),
    # and robot_execute(func) is the function call you want to happen when the button is clicked.
    # This way, robot_execute will only be executed when the button is clicked, not when start_demo is called.
    execute_button.on_click(lambda x: robot_execute(func))
    display(execute_button, output)
