"""On a base install (no numpy, no pyarrow), `roborun dreamlabs export` names the extra it needs: dreamlabs-data."""
import sys


def test_export_without_numpy_names_the_dreamlabs_data_extra(tmp_path, monkeypatch, capsys):
    from roborun_sdk import cli
    for name in ('numpy', 'pyarrow', 'pyarrow.parquet'):
        monkeypatch.setitem(sys.modules, name, None)         # import numpy -> ModuleNotFoundError, as on a base install
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path / 'home'))
    code = cli.main(['dreamlabs', 'export', '--dataset', str(tmp_path / 'ds'), '--out', str(tmp_path / 'out')])
    out = capsys.readouterr()
    text = out.out + out.err
    assert code == 2 and "roborun-sdk[dreamlabs-data]" in text and 'roborun-sdk[robot]' not in text, text
    assert not (tmp_path / 'out').exists()
