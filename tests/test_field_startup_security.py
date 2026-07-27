from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "tools" / "start_wheelchair_localization.sh"


def test_vnc_is_opt_in_and_has_no_inline_password():
    text = SCRIPT.read_text(encoding="utf-8")
    assert 'VNC="${VNC:-0}"' in text
    assert "-passwd " not in text
    assert "-rfbauth \"$VNC_AUTH\"" in text


def test_vnc_defaults_to_loopback_and_requires_an_auth_file():
    text = SCRIPT.read_text(encoding="utf-8")
    assert 'VNC_BIND="-localhost"' in text
    assert 'if [ ! -f "$VNC_AUTH" ]; then' in text
    assert "VNC_ALLOW_REMOTE" in text
