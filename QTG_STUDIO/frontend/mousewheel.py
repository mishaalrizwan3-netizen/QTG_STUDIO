"""Route wheel events to the nearest registered scroll area under the pointer."""

_NATIVE = {'Entry', 'TEntry', 'TCombobox', 'Combobox', 'Spinbox', 'TSpinbox',
           'Text', 'Listbox', 'Treeview', 'TScrollbar', 'Scrollbar'}


def install(root):
    areas = set()

    def route(event):
        widget = root.winfo_containing(event.x_root, event.y_root)
        while widget is not None:
            if widget.winfo_class() in _NATIVE:
                return None
            if widget in areas:
                if not widget.winfo_ismapped():
                    return None
                if getattr(event, 'num', None) == 4:
                    units = -1
                elif getattr(event, 'num', None) == 5:
                    units = 1
                else:
                    delta = getattr(event, 'delta', 0)
                    if not delta:
                        return None
                    units = -max(1, abs(int(delta)) // 120) if delta > 0 else max(1, abs(int(delta)) // 120)
                widget.yview_scroll(units, 'units')
                return 'break'
            widget = getattr(widget, 'master', None)
        return None

    for sequence in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
        root.bind_all(sequence, route, add='+')

    def register(widget):
        areas.add(widget)
        widget.bind('<Destroy>', lambda event: areas.discard(widget), add='+')
    return register
