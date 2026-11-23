"""Every wheel carries the docs and examples as roborun_sdk/_docs, so `roborun docs` works in any install.

The project metadata is in pyproject.toml. This only adds a build step: when the wheel is built from a checkout, README.md,
docs/*.md and examples/*.py|*.md are copied into the built package (a build that already staged src/roborun_sdk/_docs keeps
its own copy). Editable installs read them from the checkout instead.
"""
import shutil
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py

HERE = Path(__file__).resolve().parent
DOCS = (("README.md", ""), ("docs/*.md", "docs"), ("examples/*.py", "examples"), ("examples/*.md", "examples"))


class build_py_with_docs(build_py):
    def run(self):
        super().run()
        if getattr(self, "editable_mode", False):
            return
        dest = Path(self.build_lib) / "roborun_sdk" / "_docs"
        if (dest / "docs" / "BUILDING.md").is_file() or not (HERE / "docs" / "BUILDING.md").is_file():
            return
        for pattern, folder in DOCS:
            for src in sorted(HERE.glob(pattern)):
                (dest / folder).mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest / folder / src.name)


setup(cmdclass={"build_py": build_py_with_docs})
