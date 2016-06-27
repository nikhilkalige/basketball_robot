from matplotlib.backend_tools import ToolToggleBase


class Dropdown(ToolToggleBase):
    radio_group = 'dropdown'

    def __init__(self, *args, **kwargs):
        self.description = kwargs.pop('description')
        ToolToggleBase.__init__(self, *args, **kwargs)


class FirstAngles(Dropdown):
    radio_group = "firstangles"


class SecondAngles(Dropdown):
    radio_group = "secondangles"


def create_toolbar(fig, dropdown, trigger_value=""):
    for b in dropdown:
        fig.canvas.manager.toolmanager.add_tool(b, Dropdown, description=b)
        fig.canvas.manager.toolbar.add_tool(b, 'Dropdown')

    if trigger_value:
        # print("trigger", trigger_value)
        fig.canvas.manager.toolmanager.trigger_tool(trigger_value)


def create_multi_selector(fig, dropdown, trigger_values=[]):
    for b in dropdown:
        b1 = "1.{}".format(b)
        b2 = "2.{}".format(b)
        fig.canvas.manager.toolmanager.add_tool(b1, FirstAngles, description=b1)
        fig.canvas.manager.toolmanager.add_tool(b2, SecondAngles, description=b2)
        fig.canvas.manager.toolbar.add_tool(b1, 'First')
        fig.canvas.manager.toolbar.add_tool(b2, 'Second')

    if all(trigger_values):
        fig.canvas.manager.toolmanager.trigger_tool("1.{}".format(trigger_values[0]))
        fig.canvas.manager.toolmanager.trigger_tool("2.{}".format(trigger_values[1]))


def get_dropdown_value(fig, dropdown):
    for b in dropdown:
        tool = fig.canvas.manager.toolmanager.get_tool(b)
        if tool._toggled:
            # print("dropdown", tool.description)
            return tool.description

    # print("dropdown", dropdown[0])
    return dropdown[0]


def get_multiselector_value(fig, dropdown):
    b1 = ["1.{}".format(b) for b in dropdown]
    b2 = ["2.{}".format(b) for b in dropdown]

    b1_select = get_dropdown_value(fig, b1)
    b2_select = get_dropdown_value(fig, b2)
    b1_value = dropdown[b1.index(b1_select)]
    b2_value = dropdown[b2.index(b2_select)]
    if b1_value == b2_value:
        idx = b1.index(b1_select) + 1
        idx = idx if idx < len(dropdown) else (idx - len(dropdown))
        b2_value = dropdown[idx]
    return (b1_value, b2_value)
