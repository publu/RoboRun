# Security

Please report vulnerabilities privately, never in a public issue, pull request or discussion.

- GitHub's private vulnerability reporting on this repository: the Security tab, "Report a vulnerability". Only the
  maintainers see the report.
- Or email **support@roborun.dev**, with "Security" in the subject.

Include what you found, the Roborun version (`roborun --version`), how to reproduce it and what an attacker could do.
We will acknowledge the report and keep you updated until it is fixed. Please give us reasonable time to fix it before
telling anyone else.

In scope: the code in this repository, the local API on `127.0.0.1:8120` and the connection between this computer and
the Roborun service. Physical safety problems (an arm that does not stop, a rule that is not enforced) count as security
problems: report them the same way. For problems in the hosted Roborun service itself, use the same routes.
