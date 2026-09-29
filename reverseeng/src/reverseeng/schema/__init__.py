import json
from importlib import resources


def load_profile_schema() -> dict:
    return json.loads(resources.files(__name__).joinpath("device-profile.schema.json").read_text())


def validate_profile(profile: dict) -> list[str]:
    """Schema errors as readable strings; empty when valid."""
    from jsonschema import Draft202012Validator

    validator = Draft202012Validator(load_profile_schema(), format_checker=Draft202012Validator.FORMAT_CHECKER)
    return [f"{'/'.join(map(str, e.absolute_path)) or '<root>'}: {e.message}" for e in validator.iter_errors(profile)]
