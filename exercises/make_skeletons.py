#!/usr/bin/env python3
"""Generate the student skeletons from the reference solutions.

The skeletons are generated, never edited by hand. That way a skeleton can
always be completed into exactly the solution it is graded against.

How it works: each solution script marks the student's work with a TODO
region, written directly in the solution file:

        # ------------------------------------------------------------------
        # TODO 1: Short title
        #   - hint lines
        # ======================= ADD YOUR CODE BELOW =======================
        <solution code>
        # ======================= END OF YOUR CODE ==========================

For the skeleton, this script keeps the header, the hints and both marker
lines, and replaces only the solution code between the markers with

        raise NotImplementedError("TODO 1 in <file>: Short title")  # ...

It also copies each package, drops the "_solution" suffix from the package
name, copies the lab README, and says "skeleton" where the solution says
"reference solution".

It stops with an error if a marker is malformed, out of order, unbalanced,
or if the "Your tasks" list in a file's docstring does not match its TODOs.

    python3 exercises/make_skeletons.py
"""
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOL = os.path.join(HERE, "solutions")
SKEL = os.path.join(HERE, "skeletons")

RULE = "# " + "-" * 66
BEGIN = "# ======================= ADD YOUR CODE BELOW ======================="
END = "# ======================= END OF YOUR CODE =========================="
TODO_RE = re.compile(r"^( *)# TODO (\d+): (\S.*)$")
RAISE = ('raise NotImplementedError("TODO {n} in {file}: {title}")'
         '  # delete this line and write your code here')


def fail(path, lineno, why):
    sys.exit(f"make_skeletons: {path}:{lineno}: {why}")


def find_regions(lines, path):
    """Return a list of TODO regions in file order.

    Each region is a dict with n, title, indent, and the 0-based line indices
    rule, head, begin, end (the '# ---' line, the '# TODO N:' line, and the two
    marker lines). The solution code is lines[begin + 1 : end].
    """
    regions = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        m = TODO_RE.match(line)
        if m is None:
            # Any marker comment outside a well-formed region is an error.
            # (The docstring may mention the markers; only comments count.)
            if stripped.startswith("#") and ("ADD YOUR CODE BELOW" in line
                                             or "END OF YOUR CODE" in line):
                fail(path, i + 1, "marker line outside a TODO region")
            if re.match(r"^ *# *TODO\b", line):
                fail(path, i + 1, "TODO comment not in the form '# TODO N: title'")
            i += 1
            continue

        indent, n, title = m.group(1), int(m.group(2)), m.group(3).strip()
        if i == 0 or lines[i - 1] != indent + RULE:
            fail(path, i + 1, f"TODO {n}: the line above must be '{indent + RULE}'")
        if n != len(regions) + 1:
            fail(path, i + 1, f"TODO {n} found, expected TODO {len(regions) + 1}")

        # Hint lines: comments at the same indent, up to the BEGIN marker.
        j = i + 1
        while j < len(lines) and lines[j] != indent + BEGIN:
            if not lines[j].startswith(indent + "#"):
                fail(path, j + 1, f"TODO {n}: expected a hint comment or the "
                                  f"ADD YOUR CODE BELOW line at the same indent")
            if "END OF YOUR CODE" in lines[j] or TODO_RE.match(lines[j]):
                fail(path, j + 1, f"TODO {n}: no ADD YOUR CODE BELOW line")
            j += 1
        if j == len(lines):
            fail(path, i + 1, f"TODO {n}: no ADD YOUR CODE BELOW line")
        begin = j

        # Solution code, up to the END marker. No nested markers allowed.
        k = begin + 1
        while k < len(lines) and lines[k] != indent + END:
            if "ADD YOUR CODE BELOW" in lines[k] or "END OF YOUR CODE" in lines[k] \
                    or TODO_RE.match(lines[k]):
                fail(path, k + 1, f"TODO {n}: marker inside the code region "
                                  f"(missing or mis-indented END OF YOUR CODE?)")
            if lines[k].strip() and not lines[k].startswith(indent):
                fail(path, k + 1, f"TODO {n}: code is indented less than its markers")
            k += 1
        if k == len(lines):
            fail(path, begin + 1, f"TODO {n}: no END OF YOUR CODE line")
        if not any(l.strip() and not l.strip().startswith("#")
                   for l in lines[begin + 1:k]):
            fail(path, begin + 1, f"TODO {n}: no code between the markers")

        regions.append(dict(n=n, title=title, indent=indent,
                            rule=i - 1, head=i, begin=begin, end=k))
        i = k + 1
    return regions


def check_task_list(text, regions, path):
    """The module docstring must list every TODO as 'TODO N  title'."""
    for r in regions:
        entry = f"TODO {r['n']}  {r['title']}"
        if entry not in text:
            fail(path, r["head"] + 1,
                 f"the 'Your tasks' list in the docstring must contain '{entry}'")


def make_skeleton(text, path):
    """Return (skeleton text, number of TODO regions)."""
    lines = text.split("\n")
    regions = find_regions(lines, path)
    if regions:
        check_task_list(text, regions, path)
    fname = os.path.basename(path)
    # Work from the last region back, so earlier line indices stay valid.
    for r in reversed(regions):
        stub = r["indent"] + RAISE.format(n=r["n"], file=fname, title=r["title"])
        lines[r["begin"] + 1:r["end"]] = [stub]
    return "\n".join(lines), len(regions)


def main():
    if os.path.isdir(SKEL):
        shutil.rmtree(SKEL)
    os.makedirs(SKEL)

    made = 0
    for pkg_dir in sorted(os.listdir(SOL)):
        if not pkg_dir.endswith("_solution"):
            continue
        base = pkg_dir[: -len("_solution")]
        src = os.path.join(SOL, pkg_dir)
        dst = os.path.join(SKEL, base)
        shutil.copytree(src, dst,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))

        # Rename the package everywhere it appears, and cut the TODO regions
        # out of the scripts.
        for root, _, files in os.walk(dst):
            for fn in files:
                p = os.path.join(root, fn)
                with open(p) as f:
                    text = f.read()
                text = text.replace(pkg_dir, base)
                text = text.replace("Reference solution for", "Skeleton for")
                if fn.endswith(".py") and os.path.basename(root) == "scripts":
                    text, n = make_skeleton(text, os.path.relpath(
                        os.path.join(src, os.path.relpath(p, dst)), HERE))
                    if n == 0:
                        sys.exit(f"make_skeletons: {p}: no TODO regions found")
                    made += n
                    text = text.replace("reference solution", "skeleton")
                with open(p, "w") as f:
                    f.write(text)
        for fn in os.listdir(os.path.join(dst, "launch")):
            new = fn.replace("_solution", "")
            if new != fn:
                os.rename(os.path.join(dst, "launch", fn),
                          os.path.join(dst, "launch", new))

        readme = os.path.join(HERE, "readmes", base + ".md")
        if os.path.exists(readme):
            shutil.copy(readme, os.path.join(dst, "README.md"))

    print(f"skeletons regenerated: {made} TODO regions")


if __name__ == "__main__":
    main()
