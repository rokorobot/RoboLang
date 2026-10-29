"""
The RoboLang authoring benchmark (decision record 0024). Not part of the installed `robolang` package.

`lib` holds what `run.py` and `score.py` share; neither needs the network except `run.py`, and only
`run.py` ever imports the `anthropic` SDK (lazily, when it makes a client).
"""
