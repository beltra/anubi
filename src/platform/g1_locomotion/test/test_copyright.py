from ament_copyright.main import main
import pytest


# Remove the `skip` decorator once the source files have copyright headers.
@pytest.mark.skip(reason='No copyright header required for this internal package.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
