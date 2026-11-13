# Workspace organization review ? 2026-11-04

## Completed

Consolidated three outdated application documents into the workspace README. Old documents incorrectly described a scaffold, the old output directory, and a separate PID GUI. Original documents are archived; their original paths now point to the current guide.
Archived loose `.obj`, `.exp` and source-backed `.pyc` files from the application root. These are generated artifacts, not runtime DLLs or SDK link libraries. Nothing was permanently deleted.
Preserved all executable source, native DLLs, build scripts, recordings, previews, environments and existing releases.

## What must remain separate

`qtg_common_engine.py` coordinates execution; `qtg_climb_pid.py` implements climb-only PID. Combining them would increase coupling and is unnecessary for organization.
The original PID source is parsed dynamically. It must stay at its existing path.
The application's `plot_graphs.py` and `brunner/src/plot_graphs.py` implement different reporting pathways; neither is an interchangeable duplicate.
The application's `p3d_simconnect_v15.py` and BRUNNER simulator module serve existing execution paths. Identical-looking names do not establish equivalence.
Older engine modules remain imported by `main.py`, its transitive dependencies, or the PyInstaller hidden-import list. They cannot safely be removed based on age.
Root test scripts often resolve assets relative to their own file paths. Moving them requires a separate test-layout change and validation; they remain in place.

## Future consolidation candidates

Consolidate the legacy engine path into the common engine only as a separate behavior change with regression and live simulator verification.
A single reporting facade could dispatch the existing engine and climb renderers, preserving their distinct graph formats.
Build launchers can eventually be grouped under `tools/` after updating `%~dp0`, working-directory assumptions, documentation and packaging references together.
Standalone PID versions and native bridge versions remain reference implementations until call sites and independent launchers are retired.

## Disk usage and removable generated content

The main application occupies about 5.25 GB; approximately 4.49 GB is release backups. BRUNNER occupies about 1.56 GB, including its 866 MB environment. The standalone PID directory occupies about 461 MB.
These are approximate on-disk logical sizes measured before organization.
`build/`, Python caches and source-backed loose bytecode are regenerable. Build output can still contain diagnostic artifacts, so this pass only archived the loose root artifacts listed in the manifest.
`release_backups/` is the main space-saving opportunity, but contains recovery executables rather than interchangeable source snapshots. Retain until a release-retention policy is chosen.
`dist/` contains deliverables; virtual environments supply local dependencies. Preserve both.

## Audit scope

Reviewed directory contents, Python import relationships, launchers, packaging specification and existing project documentation. Binary contents, every historical CSV row and packaged library internals were not interpreted as source.
`workspace_inventory.csv` lists workspace file paths, sizes and categories, excluding Git internals, virtual environments, caches and packaged `_internal` trees. It is an inventory, not proof a file is unused.
`organization_manifest.json` records moved files, original paths and SHA-256 hashes for restoration.
No PID tuning, initialization logic, engine behavior or build was changed.

## Test-oriented layout

`test_suites/` contains one folder per catalog entry and `_shared/` for shared implementation links. Each test has an exported definition and links to existing source. Definitions are documentation, not additional runtime settings. Climb previews moved into its test folder, verified by hashes in `test_folder_moves.json`. Historical data, runtime source paths, releases and controller algorithms were preserved.
