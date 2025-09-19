import { NextResponse } from 'next/server';
import { analyzeRequirements } from '@/lib/openai';

export async function POST(request: Request) {
  try {
    const { userInput, catalogText } = await request.json();

    if (!userInput) {
      return NextResponse.json(
        { error: 'User input is required' },
        { status: 400 }
      );
    }

    const analysis = await analyzeRequirements(userInput, catalogText);

    return NextResponse.json({ analysis });
  } catch (error) {
    console.error('Error analyzing requirements:', error);
    return NextResponse.json(
      { error: 'Failed to analyze requirements' },
      { status: 500 }
    );
  }
} 