import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";
import Link from "next/link";
import { AuthProvider } from "@/components/AuthProvider";
import { SmithProfile } from "@/components/SmithProfile";
import { ProtectedNavigation, AuthenticationButton } from "@/components/ProtectedNavigation";
import { DemoModeProvider } from "@/components/DemoModeProvider";

const inter = Inter({ subsets: ["latin"] });

export const metadata: Metadata = {
  title: "Sepulki Forge - Robotics as a Service",
  description: "Design, forge, and deploy robots with Sepulki's comprehensive robotics platform",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className={inter.className}>
        <AuthProvider>
          <DemoModeProvider>
            <div className="min-h-screen bg-gray-50">
            <nav className="bg-white shadow-sm">
              <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="flex justify-between h-16">
                  <div className="flex">
                    <div className="flex-shrink-0 flex items-center">
                      <Link href="/" className="text-2xl font-bold text-orange-600">
                        🔥 Sepulki
                      </Link>
                    </div>
                    <ProtectedNavigation />
                  </div>
                  <div className="flex items-center space-x-4">
                    <SmithProfile />
                    <AuthenticationButton />
                  </div>
                </div>
              </div>
            </nav>
            <main>{children}</main>
            <footer className="bg-white border-t">
              <div className="max-w-7xl mx-auto py-6 px-4 sm:px-6 lg:px-8">
                <p className="text-center text-sm text-gray-500">
                  © 2024 Sepulki. All rights reserved.
                </p>
              </div>
            </footer>
          </div>
          </DemoModeProvider>
        </AuthProvider>
      </body>
    </html>
  );
}
