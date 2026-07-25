"""Every repository path a test reads must actually be in the repository.

Two tests once asserted against `docs/runbooks/livox-moving-localization-ko.md`
and `runtime/record_moving_localization_trial.sh`, neither of which existed here
or in any commit -- they were inherited from the NUC workspace layout when
`src/static_livox_localization/**` was imported. A clean clone could never pass.

This walks the test files statically and resolves the literal `ROOT / "a" / "b"`
path chains, so the same class of dangling reference is caught without having to
execute the suite that depends on it.
"""

import ast
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]

# Only chains rooted at one of these module-level names are checked; anything
# built from a fixture or a temporary directory is not a repository reference.
ROOT_NAMES = {"ROOT", "PROJECT", "REPOSITORY", "REPO", "PACKAGE", "HERE"}


def discover_test_files():
    for path in sorted(REPOSITORY.glob("**/test_*.py")):
        parts = set(path.parts)
        if "build" in parts or "devel" in parts or "install" in parts:
            continue
        yield path


def parents_index(node):
    """Return N for a `<...>.parents[N]` subscript, else None."""
    if not isinstance(node, ast.Subscript):
        return None
    value = node.value
    if not (isinstance(value, ast.Attribute) and value.attr == "parents"):
        return None
    index = node.slice
    if isinstance(index, ast.Index):  # Python < 3.9
        index = index.value
    if isinstance(index, ast.Constant) and isinstance(index.value, int):
        return index.value
    return None


def strip_resolve(node):
    """Unwrap `.resolve()` / `.absolute()` calls, which do not change identity here."""
    while (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"resolve", "absolute"}
    ):
        node = node.func.value
    return node


def is_path_of_dunder_file(node):
    node = strip_resolve(node)
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Path"
        and len(node.args) == 1
        and isinstance(node.args[0], ast.Name)
        and node.args[0].id == "__file__"
    )


def resolve_expression(node, known, source_file):
    """Resolve an AST expression to a filesystem path, or None if not literal."""
    node = strip_resolve(node)

    if isinstance(node, ast.Name):
        return known.get(node.id)

    if isinstance(node, ast.Attribute) and node.attr == "parent":
        base = resolve_expression(node.value, known, source_file)
        return base.parent if base else None

    index = parents_index(node)
    if index is not None:
        base = resolve_expression(node.value.value, known, source_file)
        if base is None and is_path_of_dunder_file(node.value.value):
            base = source_file
        return base.parents[index] if base else None

    if is_path_of_dunder_file(node):
        return source_file

    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
        base = resolve_expression(node.left, known, source_file)
        if base is None:
            return None
        if not (isinstance(node.right, ast.Constant) and isinstance(node.right.value, str)):
            return None
        return base / node.right.value

    return None


def module_level_roots(tree, source_file):
    known = {}
    for statement in tree.body:
        if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
            continue
        target = statement.targets[0]
        if not isinstance(target, ast.Name) or target.id not in ROOT_NAMES:
            continue
        resolved = resolve_expression(statement.value, known, source_file)
        if resolved is not None:
            known[target.id] = resolved
    return known


def referenced_paths(path):
    """Yield every literal repository path chain the test file builds."""
    source_file = path.resolve()
    tree = ast.parse(source_file.read_text(encoding="utf-8"))
    known = module_level_roots(tree, source_file)
    if not known:
        return

    for node in ast.walk(tree):
        if not (isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div)):
            continue
        # Only consider the outermost chain, i.e. skip a node that is itself the
        # left operand of another division.
        resolved = resolve_expression(node, known, source_file)
        if resolved is None:
            continue
        # A chain must end in a string literal to be a concrete reference.
        if not (isinstance(node.right, ast.Constant) and isinstance(node.right.value, str)):
            continue
        yield node.lineno, resolved


class ReferencedFilesExistTests(unittest.TestCase):
    def test_no_test_references_a_path_outside_the_repository(self):
        missing = []
        for path in discover_test_files():
            for lineno, resolved in referenced_paths(path):
                try:
                    resolved.relative_to(REPOSITORY)
                except ValueError:
                    missing.append(
                        f"{path.relative_to(REPOSITORY)}:{lineno} escapes the "
                        f"repository: {resolved}"
                    )
        self.assertEqual(missing, [], "\n".join(missing))

    def test_every_referenced_repository_path_exists(self):
        missing = []
        for path in discover_test_files():
            for lineno, resolved in referenced_paths(path):
                if not resolved.exists():
                    try:
                        shown = resolved.relative_to(REPOSITORY)
                    except ValueError:
                        continue
                    missing.append(
                        f"{path.relative_to(REPOSITORY)}:{lineno} references "
                        f"{shown}, which is not in the repository"
                    )
        self.assertEqual(missing, [], "\n".join(missing))

    def test_the_checker_actually_resolves_known_references(self):
        """Guard against the AST walk silently resolving nothing."""
        contract = REPOSITORY / "src" / "static_livox_localization" / "test"
        found = set()
        for path in (
            contract / "test_assisted_rviz_contract.py",
            contract / "test_replay_assisted_contract.py",
        ):
            found.update(resolved for _, resolved in referenced_paths(path))
        self.assertIn(REPOSITORY / "docs" / "livox_moving_localization_ko.md", found)
        self.assertIn(
            REPOSITORY / "tools" / "start_wheelchair_localization.sh", found
        )


if __name__ == "__main__":
    unittest.main()
