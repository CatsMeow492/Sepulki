# Page snapshot

```yaml
- dialog "Unhandled Runtime Error":
  - navigation:
    - button "previous" [disabled]:
      - img "previous"
    - button "next" [disabled]:
      - img "next"
    - text: 1 of 1 unhandled error Next.js (14.1.0) is outdated
    - link "(learn more)":
      - /url: https://nextjs.org/docs/messages/version-staleness
  - button "Close"
  - heading "Unhandled Runtime Error" [level=1]
  - paragraph: "ReferenceError: Link is not defined"
  - heading "Source" [level=2]
  - link "src/app/configure/page.tsx (381:13) @ Link":
    - text: src/app/configure/page.tsx (381:13) @ Link
    - img
  - text: "379 | </button> 380 | ) : ( > 381 | <Link | ^ 382 | href=\"/auth/signin\" 383 | className=\"px-6 py-3 bg-gray-600 text-white rounded-lg hover:bg-gray-700 flex items-center\" 384 | >"
  - button "Show collapsed frames"
```