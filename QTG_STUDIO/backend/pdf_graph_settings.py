"""Persistent per-test PDF graph choices; independent of simulator execution."""
import json
import os
from pathlib import Path
import re
import tempfile


def _path(root, reference):
    if not re.fullmatch(r'[0-9]+(?:\.[a-z]+)*\.\([0-9]+\)', reference):
        raise ValueError('Invalid test reference')
    return Path(root) / (reference + '.json')


def load(root, reference):
    path = _path(root, reference)
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    if (not isinstance(data, dict) or data.get('reference') != reference
            or not isinstance(data.get('selected_graphs'), list)
            or not all(isinstance(key, str) for key in data['selected_graphs'])
            or not isinstance(data.get('graph_csv_path', ''), str)):
        raise ValueError('Invalid saved PDF graph settings')
    return data


def save(root, reference, selected_graphs, graph_csv_path=''):
    path = _path(root, reference)
    if not all(isinstance(key, str) for key in selected_graphs) or not isinstance(graph_csv_path, str):
        raise ValueError('Invalid PDF graph settings')
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps({'reference': reference, 'selected_graphs': list(selected_graphs),
                          'graph_csv_path': graph_csv_path}, indent=2) + '\n'
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent,
                                         prefix=path.stem + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return path
